#!/usr/bin/env python3
from pathlib import Path
from datetime import datetime
import re
import shutil
import subprocess
import sys

root = Path.cwd()
pkg = root / "ros2_ws/src/cargo_box_detector"
pydir = pkg / "cargo_box_detector"
launch_dir = pkg / "launch"
config_dir = pkg / "config"

required = [
    root / "simulation/worlds/cargo_world.sdf",
    pydir / "fusion_node.py",
    pydir / "fsm_node.py",
    pydir / "fsm.py",
    pkg / "setup.py",
    pkg / "package.xml",
]
missing = [str(p) for p in required if not p.exists()]
if missing:
    print("Run this script from ~/cargo_box_detection. Missing:")
    print("\n".join(missing))
    sys.exit(1)

backup = root / (".cargo_nav_backup_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
backup.mkdir()
for source in required:
    target = backup / source.relative_to(root)
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)

launch_dir.mkdir(exist_ok=True)
config_dir.mkdir(exist_ok=True)

fusion_code = r'''
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException


class CargoPositionFusionNode(Node):
    def __init__(self):
        super().__init__("cargo_position_fusion_node")
        self.cloud = None
        self.cloud_received = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            PointCloud2, "/cargo_robot/depth_camera/points",
            self.cloud_callback, 10
        )
        self.create_subscription(
            String, "/cargo_box/detections", self.detection_callback, 10
        )
        self.publisher = self.create_publisher(
            String, "/cargo_box/positions", 10
        )
        self.get_logger().info(
            "Cargo position fusion started; publishing base_link and odom coordinates."
        )

    def cloud_callback(self, msg):
        self.cloud = msg
        self.cloud_received = time.monotonic()

    @staticmethod
    def transform_matrix(transform):
        t = transform.transform.translation
        q = transform.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        rotation = np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]
        ], dtype=np.float64)
        return rotation, np.array([t.x, t.y, t.z], dtype=np.float64)

    @staticmethod
    def cloud_array(msg):
        fields = {f.name: f for f in msg.fields}
        for name in ("x", "y", "z"):
            if name not in fields or fields[name].datatype != PointField.FLOAT32:
                raise ValueError(f"Missing or invalid point field: {name}")
        endian = ">" if msg.is_bigendian else "<"
        dtype = np.dtype({
            "names": ["x", "y", "z"],
            "formats": [endian+"f4"] * 3,
            "offsets": [fields["x"].offset, fields["y"].offset, fields["z"].offset],
            "itemsize": msg.point_step
        })
        return np.ndarray(
            (msg.height, msg.width), dtype=dtype, buffer=msg.data,
            strides=(msg.row_step, msg.point_step)
        )

    def get_transform(self, target, source):
        try:
            tf = self.tf_buffer.lookup_transform(target, source, Time())
            return self.transform_matrix(tf)
        except TransformException as error:
            self.get_logger().debug(f"TF unavailable {target} <- {source}: {error}")
            return None

    def detection_callback(self, message):
        try:
            result = json.loads(message.data)
            if not isinstance(result, dict):
                raise ValueError("Detection message must be a JSON object.")
            if self.cloud is None or time.monotonic() - self.cloud_received > 1.0:
                return

            cloud = self.cloud
            points = self.cloud_array(cloud)
            base_tf = self.get_transform("base_link", cloud.header.frame_id)
            odom_tf = self.get_transform("odom", cloud.header.frame_id)
            sx = cloud.width / float(result["image_width"])
            sy = cloud.height / float(result["image_height"])
            objects = []

            for obj in result.get("detections", []):
                x, y, w, h = obj["bbox"]
                x1 = max(0, int((x + 0.25*w) * sx))
                x2 = min(cloud.width, int((x + 0.75*w) * sx))
                y1 = max(0, int((y + 0.25*h) * sy))
                y2 = min(cloud.height, int((y + 0.75*h) * sy))
                if x2 <= x1 or y2 <= y1:
                    continue

                roi = points[y1:y2, x1:x2]
                xyz = np.column_stack((
                    roi["x"].ravel(), roi["y"].ravel(), roi["z"].ravel()
                )).astype(np.float64, copy=False)
                distance = np.linalg.norm(xyz, axis=1)
                xyz = xyz[
                    np.isfinite(xyz).all(axis=1)
                    & (distance > 0.1) & (distance <= 5.0)
                ]
                if len(xyz) < 5:
                    continue

                camera_xyz = np.median(xyz, axis=0)
                item = {
                    "color": str(obj.get("color", "UNKNOWN")),
                    "camera_xyz_m": [round(float(v), 4) for v in camera_xyz],
                    "distance_camera_m": round(float(np.linalg.norm(camera_xyz)), 4),
                    "base_link_xyz_m": None,
                    "distance_base_link_m": None,
                    "odom_xyz_m": None,
                    "distance_odom_m": None
                }

                if base_tf is not None:
                    rot, trans = base_tf
                    base_xyz = rot @ camera_xyz + trans
                    item["base_link_xyz_m"] = [round(float(v), 4) for v in base_xyz]
                    item["distance_base_link_m"] = round(float(np.linalg.norm(base_xyz)), 4)

                if odom_tf is not None:
                    rot, trans = odom_tf
                    odom_xyz = rot @ camera_xyz + trans
                    item["odom_xyz_m"] = [round(float(v), 4) for v in odom_xyz]
                    item["distance_odom_m"] = round(float(np.linalg.norm(odom_xyz)), 4)

                objects.append(item)

            report = {
                "frame_id": "base_link",
                "navigation_frame_id": "odom",
                "source_frame_id": cloud.header.frame_id,
                "cloud_age_s": round(time.monotonic() - self.cloud_received, 3),
                "objects": objects
            }
            output = String()
            output.data = json.dumps(report)
            self.publisher.publish(output)
            self.get_logger().info(f"Published cargo positions: {len(objects)}")
        except Exception as error:
            self.get_logger().error(f"Position fusion failed: {error}")


def main(args=None):
    rclpy.init(args=args)
    node = CargoPositionFusionNode()
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
'''

navigation_code = r'''
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
        self.declare_parameter("goal_frame", "odom")
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
                    f"Navigating to drop-off ({self.dropoff_x:.2f}, {self.dropoff_y:.2f}) in odom."
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
                    xyz = item.get("odom_xyz_m")
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
                self.goal_active = False
                self.publish_failure(f"Nav2 rejected the {self.goal_kind} goal.")
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
'''

launch_code = r'''
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("cargo_box_detector")
    nav2_share = get_package_share_directory("nav2_bringup")
    map_file = os.path.join(package_share, "config", "warehouse_static.yaml")
    params_file = os.path.join(package_share, "config", "nav2_params.yaml")
    nav2_launch = os.path.join(nav2_share, "launch", "navigation_launch.py")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "yaml_filename": map_file,
            "frame_id": "odom",
            "use_sim_time": True
        }]
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            "node_names": ["map_server"]
        }]
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": params_file
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "image_topic",
            default_value="/cargo_robot/front_camera",
            description="Gazebo RGB camera image topic"
        ),
        map_server,
        map_lifecycle,
        nav2,
        Node(
            package="cargo_box_detector",
            executable="cargo_box_detector_node",
            name="cargo_box_detector_node",
            output="screen",
            parameters=[{"image_topic": LaunchConfiguration("image_topic")}]
        ),
        Node(
            package="cargo_box_detector",
            executable="cargo_position_fusion_node",
            name="cargo_position_fusion_node",
            output="screen",
            parameters=[{"use_sim_time": True}]
        ),
        Node(
            package="cargo_box_detector",
            executable="cargo_delivery_fsm_node",
            name="cargo_delivery_fsm_node",
            output="screen"
        ),
        Node(
            package="cargo_box_detector",
            executable="cargo_navigation_node",
            name="cargo_navigation_node",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "goal_frame": "odom",
                "approach_distance": 1.05,
                "dropoff_x": 3.5,
                "dropoff_y": 0.0,
                "dropoff_yaw": 0.0,
                "pickup_delay_s": 2.0,
                "dropoff_delay_s": 2.0
            }]
        ),
        Node(
            package="cargo_box_detector",
            executable="cargo_metrics_node",
            name="cargo_metrics_node",
            output="screen"
        )
    ])
'''

(pydir / "fusion_node.py").write_text(fusion_code, encoding="utf-8")
(pydir / "navigation_node.py").write_text(navigation_code, encoding="utf-8")
(launch_dir / "cargo_box_system.launch.py").write_text(launch_code, encoding="utf-8")

# Add yellow to the existing HSV detector.
detector_path = pydir / "detector.py"
if detector_path.exists():
    detector_text = detector_path.read_text(encoding="utf-8")
    if "YELLOW" not in detector_text:
        detector_text = detector_text.replace(
            '    "BLUE": [\n        ((95, 80, 60), (135, 255, 255))\n    ]\n}',
            '    "BLUE": [\n        ((95, 80, 60), (135, 255, 255))\n    ],\n'
            '    "YELLOW": [\n        ((20, 80, 80), (35, 255, 255))\n    ]\n}'
        )
        detector_text = detector_text.replace(
            '    "BLUE": (255, 0, 0)\n}',
            '    "BLUE": (255, 0, 0),\n    "YELLOW": (0, 255, 255)\n}'
        )
        detector_path.write_text(detector_text, encoding="utf-8")
else:
    print("NOTE: detector.py not found; skipped automatic yellow-color addition.")

# Register the new executable and install config files.
setup_path = pkg / "setup.py"
setup_text = setup_path.read_text(encoding="utf-8")
if "glob('config/*')" not in setup_text:
    setup_text = setup_text.replace(
        "        (\n            os.path.join('share', package_name, 'launch'),\n            glob('launch/*.launch.py')\n        ),",
        "        (\n            os.path.join('share', package_name, 'launch'),\n            glob('launch/*.launch.py')\n        ),\n"
        "        (\n            os.path.join('share', package_name, 'config'),\n            glob('config/*')\n        ),"
    )
if "cargo_navigation_node =" not in setup_text:
    setup_text = setup_text.replace(
        "'cargo_position_fusion_node = cargo_box_detector.fusion_node:main',",
        "'cargo_position_fusion_node = cargo_box_detector.fusion_node:main',\n"
        "            'cargo_navigation_node = cargo_box_detector.navigation_node:main',"
    )
setup_path.write_text(setup_text, encoding="utf-8")

package_path = pkg / "package.xml"
package_text = package_path.read_text(encoding="utf-8")
for dep in [
    "geometry_msgs", "action_msgs", "nav2_msgs", "nav2_map_server",
    "nav2_lifecycle_manager", "nav2_bringup", "ament_index_python"
]:
    tag = f"<exec_depend>{dep}</exec_depend>"
    if tag not in package_text:
        package_text = package_text.replace(
            "  <test_depend>ament_copyright</test_depend>",
            f"  {tag}\n  <test_depend>ament_copyright</test_depend>"
        )
package_path.write_text(package_text, encoding="utf-8")

# Rasterize the wall and cargo-box geometry from cargo_world.sdf.
res = 0.05
xmin, xmax, ymin, ymax = -5.5, 5.5, -4.5, 4.5
width = round((xmax - xmin) / res)
height = round((ymax - ymin) / res)
grid = [[254 for _ in range(width)] for _ in range(height)]

def rectangle(cx, cy, sx, sy):
    ix0 = max(0, int((cx - sx/2 - xmin) / res))
    ix1 = min(width, int((cx + sx/2 - xmin) / res) + 1)
    iy0 = max(0, int((cy - sy/2 - ymin) / res))
    iy1 = min(height, int((cy + sy/2 - ymin) / res) + 1)
    for iy in range(iy0, iy1):
        row = height - 1 - iy
        for ix in range(ix0, ix1):
            grid[row][ix] = 0

for values in [
    (0.0, 4.0, 10.2, 0.2),
    (0.0, -4.0, 10.2, 0.2),
    (5.0, 0.0, 0.2, 8.2),
    (-5.0, 0.0, 0.2, 8.2),
    (-1.5, 1.5, 0.65, 0.65),
    (-1.5, -1.5, 0.65, 0.65),
    (0.0, 2.4, 0.65, 0.65),
    (0.0, -2.4, 0.65, 0.65),
]:
    rectangle(*values)

with (config_dir / "warehouse_static.pgm").open("w", encoding="ascii") as f:
    f.write(f"P2\n{width} {height}\n255\n")
    for row in grid:
        f.write(" ".join(map(str, row)) + "\n")

(config_dir / "warehouse_static.yaml").write_text(
    "image: warehouse_static.pgm\n"
    "mode: trinary\n"
    f"resolution: {res:.2f}\n"
    f"origin: [{xmin:.2f}, {ymin:.2f}, 0.0]\n"
    "negate: 0\n"
    "occupied_thresh: 0.65\n"
    "free_thresh: 0.25\n",
    encoding="utf-8"
)

# Adapt the installed Nav2 defaults for odom frame, base_link and Gazebo's Twist topic.
try:
    prefix = subprocess.check_output(
        ["ros2", "pkg", "prefix", "nav2_bringup"], text=True
    ).strip()
    default_params = Path(prefix) / "share/nav2_bringup/params/nav2_params.yaml"
    if not default_params.exists():
        raise FileNotFoundError(str(default_params))
    params = default_params.read_text(encoding="utf-8")
    params = params.replace("base_footprint", "base_link")
    params = re.sub(r"(?m)^(\s*global_frame:\s*)map\s*$", r"\1odom", params)
    params = re.sub(
        r"(?mi)^(\s*enable_stamped_cmd_vel:\s*)true\b", r"\1false", params
    )
    params = re.sub(r"(?m)^(\s*topic:\s*)scan\s*$", r"\1/scan", params)
    (config_dir / "nav2_params.yaml").write_text(params, encoding="utf-8")
except Exception as error:
    print(f"Could not create Nav2 parameters: {error}")
    print(f"Backup is at: {backup}")
    sys.exit(2)

print("PATCH APPLIED.")
print(f"Backup: {backup}")
print(f"Static map: {config_dir / 'warehouse_static.yaml'}")
print(f"Nav2 params: {config_dir / 'nav2_params.yaml'}")
print("The drop-off point defaults to odom (3.5, 0.0).")
print("Pickup/drop-off are timed simulation events; no physical gripper is implemented.")
print("The generated static map assumes odom origin matches the Gazebo world origin.")
