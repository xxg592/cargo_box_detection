#!/usr/bin/env python3
"""Offline cargo-box detection demo.

This runs exactly the same OpenCV code as the ROS 2 node
(``cargo_box_detector.detector.detect_boxes``), but without ROS or Gazebo,
so the detector can be graded on a laptop and reproduced from a screenshot.

Usage
-----
    # run from ~/cargo_box_detection
    PYTHONPATH=ros2_ws/src/cargo_box_detector python3 src/detect_color_boxes.py \
        images/test_boxes.png images/test_empty.png cargo_camera_original.png

Outputs
-------
    results/<image>_detected.png     annotated image (bounding boxes + labels)
    results/<image>_detection.json   the same payload the ROS node publishes
    stdout                           per-colour status and box centres
"""

import argparse
import json
import sys
from pathlib import Path

import cv2

from cargo_box_detector.detector import detect_boxes


def describe(result):
    """Human-readable summary of one detection result."""
    lines = []
    status = result["status"]
    lines.append(
        f"status={status} detected_count={result['detected_count']} "
        f"({result['image_width']}x{result['image_height']} px)"
    )
    for colour, found in result["color_status"].items():
        lines.append(f"  {colour:<6} {'DETECTED' if found else 'not detected'}")
    for detection in result["detections"]:
        x, y, w, h = detection["bbox"]
        cx, cy = detection["center"]
        lines.append(
            f"  -> {detection['color']:<6} bbox=({x},{y},{w},{h}) "
            f"centre=({cx},{cy}) area={detection['area']:.0f} px^2"
        )
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="OpenCV cargo-box detection (offline demo)"
    )
    parser.add_argument("images", nargs="+", help="input image paths")
    parser.add_argument(
        "--output-dir", default="results",
        help="directory for annotated images and JSON (default: results)",
    )
    parser.add_argument(
        "--json", action="store_true",
        help="print the full JSON payload for every image",
    )
    args = parser.parse_args(argv)

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    exit_code = 0

    for name in args.images:
        path = Path(name)

        if not path.exists():
            print(f"ERROR: no such image: {path}")
            exit_code = 1
            continue

        result, annotated = detect_boxes(path)

        print(f"\n=== {path} ===")
        print(describe(result))

        image_out = output_dir / f"{path.stem}_detected.png"
        if not cv2.imwrite(str(image_out), annotated):
            print(f"ERROR: cannot write {image_out}")
            exit_code = 1
        else:
            print(f"saved: {image_out}")

        json_out = output_dir / f"{path.stem}_detection.json"
        json_out.write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(f"saved: {json_out}")

        if args.json:
            print(json.dumps(result, indent=2))

    return exit_code


if __name__ == "__main__":
    sys.exit(main())