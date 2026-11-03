import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from cargo_box_detector.fsm import CargoDeliveryFSM


class CargoDeliveryFSMNode(Node):

    def __init__(self):
        super().__init__("cargo_delivery_fsm_node")

        self.fsm = CargoDeliveryFSM()

        self.state_publisher = self.create_publisher(
            String,
            "/cargo_box/mission_state",
            10
        )

        self.detection_subscription = self.create_subscription(
            String,
            "/cargo_box/detections",
            self.detection_callback,
            10
        )

        self.event_subscription = self.create_subscription(
            String,
            "/cargo_box/mission_event",
            self.event_callback,
            10
        )

        self.timer = self.create_timer(
            1.0,
            self.publish_heartbeat
        )

        self.publish_state("NODE_STARTED")

        self.get_logger().info(
            "Cargo delivery FSM node started."
        )

        self.get_logger().info(
            "Waiting for cargo detection messages."
        )

    def publish_state(self, event):

        payload = {
            "state": self.fsm.state.value,
            "event": event,
            "last_transition": (
                self.fsm.history[-1]
                if self.fsm.history else None
            ),
            "target_detection": self.fsm.target_detection,
            "failure_reason": self.fsm.failure_reason
        }

        message = String()
        message.data = json.dumps(payload)

        self.state_publisher.publish(message)

    def publish_heartbeat(self):

        self.publish_state("STATE_HEARTBEAT")

    def detection_callback(self, message):

        try:
            result = json.loads(message.data)

            if not isinstance(result, dict):
                raise ValueError(
                    "Detection message must contain a JSON object."
                )

            previous_state = self.fsm.state

            self.fsm.on_detection(result)

            if self.fsm.state != previous_state:

                event = self.fsm.history[-1]["event"]

                self.publish_state(event)

                self.get_logger().info(
                    f"Mission state changed: "
                    f"{previous_state.value} -> "
                    f"{self.fsm.state.value}"
                )

        except (
            json.JSONDecodeError,
            TypeError,
            ValueError
        ) as error:

            self.get_logger().error(
                f"Invalid detection message: {error}"
            )

    def event_callback(self, message):

        event = message.data.strip().upper()

        try:
            previous_state = self.fsm.state

            if event == "NAVIGATION_SUCCEEDED":
                self.fsm.on_navigation_succeeded()

            elif event == "PICKUP_SUCCEEDED":
                self.fsm.on_pickup_succeeded()

            elif event == "DROPOFF_SUCCEEDED":
                self.fsm.on_dropoff_succeeded()

            elif event == "EXPLORATION_FINISHED":
                self.fsm.on_exploration_finished()

            elif event.startswith("OPERATION_FAILED:"):
                reason = message.data.split(":", 1)[1].strip()
                self.fsm.on_operation_failed(
                    reason or "Unspecified failure"
                )

            else:
                self.get_logger().warning(
                    f"Unknown mission event: {event}"
                )
                return

            if self.fsm.state != previous_state:

                transition_event = self.fsm.history[-1]["event"]

                self.publish_state(transition_event)

                self.get_logger().info(
                    f"Mission state changed: "
                    f"{previous_state.value} -> "
                    f"{self.fsm.state.value}"
                )

        except (RuntimeError, ValueError) as error:

            self.get_logger().error(
                f"Mission event rejected: {error}"
            )


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:
        node = CargoDeliveryFSMNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
