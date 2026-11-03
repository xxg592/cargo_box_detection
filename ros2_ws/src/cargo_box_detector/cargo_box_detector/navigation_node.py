
import json
import math
import time
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException


class CargoNavigationNode(Node):
    def __init__(self):
        super().__init__("cargo_navigation_node")
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("approach_distance", 1.05)
        self.declare_parameter("dropoff_x", 3.5)
        self.declare_parameter("dropoff_y", 0.0)
        self.declare_parameter("dropoff_yaw", 0.0)
        self.declare_parameter("pickup_delay_s", 2.0)
        self.declare_parameter("dropoff_delay_s", 2.0)
        self.declare_parameter("navigation_wait_s", 45.0)

        self.goal_frame = self.get_parameter("goal_frame").value
        self.approach_distance = float(self.get_parameter("approach_distance").value)
        self.dropoff_x = float(self.get_parameter("dropoff_x").value)
        self.dropoff_y = float(self.get_parameter("dropoff_y").value)
        self.dropoff_yaw = float(self.get_parameter("dropoff_yaw").value)
        self.pickup_delay = float(self.get_parameter("pickup_delay_s").value)
        self.dropoff_delay = float(self.get_parameter("dropoff_delay_s").value)
        self.navigation_wait = float(self.get_parameter("navigation_wait_s").value)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.mission_state = "EXPLORING"
        self.target_color = None
        self.positions = []
        self.positions_received = 0.0
        self.state_started = time.monotonic()
        self.wait_started = None
        self.operation_done = False
        self.goal_active = False
        self.goal_kind = None

        self.create_subscription(
            String, "/cargo_box/mission_state", self.state_callback, 10
        )
        self.create_subscription(
            String, "/cargo_box/positions", self.positions_callback, 10
        )
        self.event_publisher = self.create_publisher(
            String, "/cargo_box/mission_event", 10
        )
        self.timer = self.create_timer(0.25, self.timer_callback)
        self.get_logger().info("Cargo navigation coordinator started.")

    def state_callback(self, message):
        try:
            payload = json.loads(message.data)
            state = payload.get("state", "UNKNOWN")
            if state == self.mission_state:
                return
            previous = self.mission_state
            self.mission_state = state
            self.state_started = time.monotonic()
            self.wait_started = None
            self.operation_done = False
            if state == "NAVIGATING_TO_BOX":
                target = payload.get("target_detection") or {}
                self.target_color = target.get("color")
                self.get_logger().info(f"Navigating to {self.target_color} cargo.")
            elif state == "NAVIGATING_TO_DROPOFF":
                self.get_logger().info(
                    f"Navigating to drop-off ({self.dropoff_x:.2f}, {self.dropoff_y:.2f}) "
                    f"in {self.goal_frame}."
                )
            self.get_logger().info(f"Mission state: {previous} -> {state}")
        except (json.JSONDecodeError, TypeError, AttributeError) as error:
            self.get_logger().error(f"Invalid mission state: {error}")

    def positions_callback(self, message):
        try:
            self.positions = json.loads(message.data).get("objects", [])
            self.positions_received = time.monotonic()
        except (json.JSONDecodeError, TypeError, AttributeError) as error:
            self.get_logger().warning(f"Invalid cargo positions: {error}")

    def timer_callback(self):
        now = time.monotonic()

        if self.mission_state == "NAVIGATING_TO_BOX":
            if self.goal_active:
                return
            if self.wait_started is None:
                self.wait_started = now
            if now - self.wait_started > self.navigation_wait:
                self.publish_failure("Timed out waiting for fresh 3D cargo position or Nav2.")
                return
            if not self.nav_client.wait_for_server(timeout_sec=0.05):
                return
            if now - self.positions_received > 1.5:
                return

            target = None
            for item in self.positions:
                if item.get("color") == self.target_color:
                    xyz = item.get("map_xyz_m") or item.get("odom_xyz_m")
                    if isinstance(xyz, list) and len(xyz) == 3:
                        try:
                            values = [float(v) for v in xyz]
                            if all(math.isfinite(v) for v in values):
                                target = values
                                break
                        except (TypeError, ValueError):
                            pass
            if target is None:
                return

            robot_xy = self.get_robot_xy()
            if robot_xy is None:
                return
            dx, dy = target[0] - robot_xy[0], target[1] - robot_xy[1]
            distance = math.hypot(dx, dy)
            if distance < 0.05:
                self.publish_failure("Cargo target has invalid or too-close coordinates.")
                return
            travel = max(0.0, distance - self.approach_distance)
            goal_x = robot_xy[0] + dx / distance * travel
            goal_y = robot_xy[1] + dy / distance * travel
            goal_yaw = math.atan2(target[1] - goal_y, target[0] - goal_x)
            self.send_goal(goal_x, goal_y, goal_yaw, "cargo")
            return

        if self.mission_state == "PICKING_UP":
            if not self.operation_done and now - self.state_started >= self.pickup_delay:
                self.operation_done = True
                self.publish_event("PICKUP_SUCCEEDED")
                self.get_logger().info("Simulated pickup completed.")
            return

        if self.mission_state == "NAVIGATING_TO_DROPOFF":
            if self.goal_active:
                return
            if self.wait_started is None:
                self.wait_started = now
            if now - self.wait_started > self.navigation_wait:
                self.publish_failure("Timed out waiting for Nav2 drop-off navigation.")
                return
            if self.nav_client.wait_for_server(timeout_sec=0.05):
                self.send_goal(
                    self.dropoff_x, self.dropoff_y, self.dropoff_yaw, "dropoff"
                )
            return

        if self.mission_state == "DROPPING_OFF":
            if not self.operation_done and now - self.state_started >= self.dropoff_delay:
                self.operation_done = True
                self.publish_event("DROPOFF_SUCCEEDED")
                self.get_logger().info("Simulated drop-off completed.")

    def get_robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.goal_frame, "base_link", Time()
            )
            return (
                float(tf.transform.translation.x),
                float(tf.transform.translation.y)
            )
        except TransformException as error:
            self.get_logger().debug(f"Waiting for robot TF: {error}")
            return None

    def send_goal(self, x, y, yaw, kind):
        if self.goal_active:
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.goal_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_active = True
        self.goal_kind = kind
        self.get_logger().info(
            f"Sending {kind} goal: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f} deg"
        )
        self.nav_client.send_goal_async(goal).add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                # bt_navigator may still be activating; retrying on the next
                # tick is bounded by navigation_wait_s.
                self.goal_active = False
                self.get_logger().warning(
                    f"Nav2 rejected the {self.goal_kind} goal; retrying."
                )
                return
            self.get_logger().info(f"Nav2 accepted the {self.goal_kind} goal.")
            handle.get_result_async().add_done_callback(
                self.navigation_result_callback
            )
        except Exception as error:
            self.goal_active = False
            self.publish_failure(f"Navigation goal error: {error}")

    def navigation_result_callback(self, future):
        kind = self.goal_kind
        self.goal_active = False
        self.goal_kind = None
        try:
            result = future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f"Nav2 reached the {kind} goal.")
                self.publish_event("NAVIGATION_SUCCEEDED")
            else:
                self.publish_failure(
                    f"Nav2 {kind} navigation failed with status {result.status}."
                )
        except Exception as error:
            self.publish_failure(f"Navigation result error: {error}")

    def publish_event(self, event):
        message = String()
        message.data = event
        self.event_publisher.publish(message)

    def publish_failure(self, reason):
        if self.operation_done:
            return
        self.operation_done = True
        self.get_logger().error(reason)
        self.publish_event(f"OPERATION_FAILED: {reason}")


def main(args=None):
    rclpy.init(args=args)
    node = CargoNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
