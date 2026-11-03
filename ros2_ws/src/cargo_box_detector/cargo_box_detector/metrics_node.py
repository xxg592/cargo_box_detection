import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CargoMetricsNode(Node):

    TERMINAL_STATES = {
        "COMPLETED",
        "FAILED",
        "NO_BOX_FOUND"
    }

    def __init__(self):
        super().__init__("cargo_metrics_node")

        self.detection_message_count = 0
        self.detected_frames = 0
        self.empty_frames = 0
        self.total_detection_observations = 0

        self.color_observations = {
            "RED": 0,
            "GREEN": 0,
            "BLUE": 0,
            "YELLOW": 0
        }

        self.latency_count = 0
        self.latency_sum = 0.0
        self.latency_min = None
        self.latency_max = None

        self.mission_start_time = None
        self.mission_end_time = None
        self.current_state = "UNKNOWN"
        self.final_state = None

        self.state_transitions = []
        self.final_report_logged = False

        self.detection_subscription = self.create_subscription(
            String,
            "/cargo_box/detections",
            self.detection_callback,
            10
        )

        self.state_subscription = self.create_subscription(
            String,
            "/cargo_box/mission_state",
            self.state_callback,
            10
        )

        self.metrics_publisher = self.create_publisher(
            String,
            "/cargo_box/metrics",
            10
        )

        self.timer = self.create_timer(
            2.0,
            self.publish_metrics
        )

        self.get_logger().info(
            "Cargo metrics node started."
        )

        self.get_logger().info(
            "Waiting for detection and mission data."
        )

    def detection_callback(self, message):

        try:
            result = json.loads(message.data)

            if not isinstance(result, dict):
                raise ValueError(
                    "Detection message must be a JSON object."
                )

            detected_count = max(
                0,
                int(result.get("detected_count", 0))
            )

            self.detection_message_count += 1

            if detected_count > 0:
                self.detected_frames += 1
            else:
                self.empty_frames += 1

            self.total_detection_observations += detected_count

            detections = result.get("detections", [])

            if isinstance(detections, list):
                for detection in detections:
                    if not isinstance(detection, dict):
                        continue

                    color = str(
                        detection.get("color", "UNKNOWN")
                    ).upper()

                    self.color_observations.setdefault(color, 0)
                    self.color_observations[color] += 1

            latency = result.get("detection_time_ms")

            if latency is not None:
                latency = float(latency)

                if latency >= 0:
                    self.latency_count += 1
                    self.latency_sum += latency

                    if self.latency_min is None:
                        self.latency_min = latency
                        self.latency_max = latency
                    else:
                        self.latency_min = min(
                            self.latency_min,
                            latency
                        )
                        self.latency_max = max(
                            self.latency_max,
                            latency
                        )

        except (
            json.JSONDecodeError,
            TypeError,
            ValueError
        ) as error:

            self.get_logger().error(
                f"Invalid detection data: {error}"
            )

    def state_callback(self, message):

        try:
            result = json.loads(message.data)

            if not isinstance(result, dict):
                raise ValueError(
                    "Mission state must be a JSON object."
                )

            state = str(result.get("state", "UNKNOWN"))
            now = time.monotonic()

            if self.mission_start_time is None:
                self.mission_start_time = now

            if state != self.current_state:

                if self.current_state != "UNKNOWN":

                    transition = result.get("last_transition")

                    if isinstance(transition, dict):
                        self.state_transitions.append(
                            transition.copy()
                        )

                self.current_state = state

            if state in self.TERMINAL_STATES:

                self.final_state = state

                if self.mission_end_time is None:
                    self.mission_end_time = now

                self.publish_metrics()

                if not self.final_report_logged:
                    report = self.build_metrics()

                    self.get_logger().info(
                        "FINAL METRICS: "
                        + json.dumps(report)
                    )

                    self.final_report_logged = True

        except (
            json.JSONDecodeError,
            TypeError,
            ValueError
        ) as error:

            self.get_logger().error(
                f"Invalid mission state: {error}"
            )

    def build_metrics(self):

        now = time.monotonic()

        duration = 0.0

        if self.mission_start_time is not None:

            end_time = (
                self.mission_end_time
                if self.mission_end_time is not None
                else now
            )

            duration = max(
                0.0,
                end_time - self.mission_start_time
            )

        average_latency = (
            self.latency_sum / self.latency_count
            if self.latency_count > 0
            else None
        )

        return {
            "mission_state": self.current_state,
            "mission_result": self.final_state or "IN_PROGRESS",
            "mission_completed": self.final_state == "COMPLETED",
            "mission_duration_s": round(duration, 3),
            "detection_messages": self.detection_message_count,
            "detection_rate": (
                round(
                    self.detected_frames / self.detection_message_count,
                    4
                )
                if self.detection_message_count > 0
                else None
            ),
            "frames_with_detections": self.detected_frames,
            "frames_without_detections": self.empty_frames,
            "total_detection_observations": (
                self.total_detection_observations
            ),
            "color_observations": self.color_observations,
            "detection_latency_ms": {
                "count": self.latency_count,
                "average": (
                    round(average_latency, 3)
                    if average_latency is not None
                    else None
                ),
                "minimum": self.latency_min,
                "maximum": self.latency_max
            },
            "state_transition_count": len(
                self.state_transitions
            ),
            "state_transitions": self.state_transitions
        }

    def publish_metrics(self):

        if self.mission_start_time is None:
            return

        report = self.build_metrics()

        message = String()
        message.data = json.dumps(report)

        self.metrics_publisher.publish(message)


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:
        node = CargoMetricsNode()
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
