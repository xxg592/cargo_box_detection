import json
import time

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from cargo_box_detector.detector import detect_boxes


class CargoBoxDetectorNode(Node):

    def __init__(self):
        super().__init__("cargo_box_detector_node")

        self.declare_parameter(
            "image_topic",
            "/cargo_robot/front_camera"
        )

        self.image_topic = (
            self.get_parameter("image_topic")
            .get_parameter_value()
            .string_value
        )

        if not self.image_topic:
            raise ValueError("Image topic cannot be empty.")

        self.publisher = self.create_publisher(
            String,
            "/cargo_box/detections",
            10
        )

        self.declare_parameter(
            "debug_image_topic",
            "/cargo_box/debug_image"
        )

        self.debug_image_topic = (
            self.get_parameter("debug_image_topic")
            .get_parameter_value()
            .string_value
        )

        self.debug_publisher = self.create_publisher(
            Image,
            self.debug_image_topic,
            10
        )

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.frame_count = 0

        self.get_logger().info(
            "Real-time cargo detector started."
        )

        self.get_logger().info(
            f"Camera topic: {self.image_topic}"
        )

    def image_to_bgr(self, message):

        height = message.height
        width = message.width
        step = message.step

        if height <= 0 or width <= 0:
            raise ValueError("Invalid image dimensions.")

        channels_by_encoding = {
            "bgr8": 3,
            "rgb8": 3,
            "bgra8": 4,
            "rgba8": 4,
            "mono8": 1
        }

        encoding = message.encoding.lower()

        if encoding not in channels_by_encoding:
            raise ValueError(
                f"Unsupported image encoding: {encoding}"
            )

        channels = channels_by_encoding[encoding]

        if step < width * channels:
            raise ValueError("Invalid image row step.")

        required_size = height * step

        data = np.frombuffer(
            message.data,
            dtype=np.uint8
        )

        if data.size < required_size:
            raise ValueError("Incomplete image data.")

        rows = data[:required_size].reshape(
            height,
            step
        )

        pixels = rows[:, :width * channels]

        if channels == 1:

            image = pixels.reshape(height, width)

            return cv2.cvtColor(
                image,
                cv2.COLOR_GRAY2BGR
            )

        image = pixels.reshape(
            height,
            width,
            channels
        )

        if encoding == "rgb8":
            return cv2.cvtColor(
                image,
                cv2.COLOR_RGB2BGR
            )

        if encoding == "rgba8":
            return cv2.cvtColor(
                image,
                cv2.COLOR_RGBA2BGR
            )

        if encoding == "bgra8":
            return cv2.cvtColor(
                image,
                cv2.COLOR_BGRA2BGR
            )

        return image.copy()

    def image_callback(self, message):

        start_time = time.perf_counter()

        try:
            image = self.image_to_bgr(message)

            result, annotated = detect_boxes(
                image,
                source=self.image_topic
            )

            detection_time = (
                time.perf_counter() - start_time
            ) * 1000.0

            result["detection_time_ms"] = round(
                detection_time,
                3
            )

            result["frame_id"] = message.header.frame_id

            output = String()
            output.data = json.dumps(result)

            self.publisher.publish(output)

            if self.debug_publisher.get_subscription_count() > 0:
                self.publish_debug_image(annotated, message.header)

            self.frame_count += 1

            self.get_logger().info(
                f"Frame {self.frame_count} | "
                f"{result['status']} | "
                f"detected_count={result['detected_count']} | "
                f"detection_time_ms={detection_time:.2f}"
            )

        except Exception as error:

            self.get_logger().error(
                f"Image processing failed: {error}"
            )


    def publish_debug_image(self, annotated, header):
        """Publish the annotated BGR frame for RViz / rqt visualisation."""

        message = Image()
        message.header.stamp = header.stamp
        message.header.frame_id = header.frame_id
        message.height, message.width = annotated.shape[:2]
        message.encoding = "bgr8"
        message.is_bigendian = 0
        message.step = message.width * 3
        message.data = annotated.tobytes()
        self.debug_publisher.publish(message)


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:
        node = CargoBoxDetectorNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except (ValueError, TypeError) as error:
        print(f"ERROR: {error}")

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
