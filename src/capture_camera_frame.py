#!/usr/bin/env python3
"""Grab a single frame from a ROS 2 camera topic and save it as PNG.

Used to turn a live Gazebo camera stream into a still image that the
detector demo can be re-run on offline.

Usage
-----
    source /opt/ros/jazzy/setup.bash
    python3 src/capture_camera_frame.py \
        --topic /cargo_robot/front_camera --output images/gazebo_frame.png
"""

import argparse
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

CHANNELS = {"bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4, "mono8": 1}


class FrameGrabber(Node):

    def __init__(self, topic, output):
        super().__init__("camera_frame_grabber")
        self.output = output
        self.saved = False
        self.subscription = self.create_subscription(
            Image, topic, self.callback, 1
        )

    def callback(self, message):
        if self.saved:
            return

        encoding = message.encoding.lower()
        channels = CHANNELS.get(encoding)
        if channels is None:
            self.get_logger().error(f"unsupported encoding: {message.encoding}")
            return

        raw = np.frombuffer(message.data, dtype=np.uint8)
        rows = raw[: message.height * message.step].reshape(
            message.height, message.step
        )
        pixels = rows[:, : message.width * channels]

        if channels == 1:
            image = cv2.cvtColor(
                pixels.reshape(message.height, message.width),
                cv2.COLOR_GRAY2BGR,
            )
        else:
            image = pixels.reshape(message.height, message.width, channels)
            if encoding == "rgb8":
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            elif encoding == "rgba8":
                image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
            elif encoding == "bgra8":
                image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
            else:
                image = image.copy()

        if not cv2.imwrite(self.output, image):
            self.get_logger().error(f"cannot write {self.output}")
            return

        self.saved = True
        self.get_logger().info(
            f"saved {self.output} ({message.width}x{message.height}, "
            f"{message.encoding}, frame_id='{message.header.frame_id}')"
        )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/cargo_robot/front_camera")
    parser.add_argument("--output", default="images/gazebo_frame.png")
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args(argv)

    rclpy.init()
    node = FrameGrabber(args.topic, args.output)
    deadline = time.monotonic() + args.timeout

    try:
        while rclpy.ok() and not node.saved and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not node.saved:
        print(f"ERROR: no frame received on {args.topic} within {args.timeout}s")
        return 1

    print(f"SAVED {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())