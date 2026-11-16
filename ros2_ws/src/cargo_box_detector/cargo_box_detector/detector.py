from pathlib import Path

import cv2
import numpy as np


COLOR_RANGES = {
    "RED": [
        ((0, 100, 80), (10, 255, 255)),
        ((170, 100, 80), (180, 255, 255))
    ],
    "GREEN": [
        ((40, 80, 60), (85, 255, 255))
    ],
    "BLUE": [
        ((95, 80, 60), (135, 255, 255))
    ],
    "YELLOW": [
        ((20, 80, 80), (35, 255, 255))
    ]
}


DRAW_COLORS = {
    "RED": (0, 0, 255),
    "GREEN": (0, 200, 0),
    "BLUE": (255, 0, 0),
    "YELLOW": (0, 255, 255)
}


def detect_boxes(image_input, source="camera"):

    image_path = None

    if isinstance(image_input, (str, Path)):

        image_path = Path(image_input).expanduser()
        image = cv2.imread(str(image_path))

        if image is None:
            raise FileNotFoundError(
                f"Cannot read image: {image_path}"
            )

        source = str(image_path)

    elif isinstance(image_input, np.ndarray):

        image = image_input.copy()

    else:
        raise TypeError("Unsupported image input.")

    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("Expected a three-channel BGR image.")

    image_height, image_width = image.shape[:2]

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    detections = []

    color_status = {
        color_name: False
        for color_name in COLOR_RANGES
    }

    kernel = np.ones((5, 5), np.uint8)

    for color_name, ranges in COLOR_RANGES.items():

        mask = np.zeros(
            hsv.shape[:2],
            dtype=np.uint8
        )

        for lower, upper in ranges:

            lower = np.array(lower, dtype=np.uint8)
            upper = np.array(upper, dtype=np.uint8)

            color_mask = cv2.inRange(
                hsv,
                lower,
                upper
            )

            mask = cv2.bitwise_or(
                mask,
                color_mask
            )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        boxes = []

        for contour in contours:

            area = cv2.contourArea(contour)

            if area < 1000:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            boxes.append((x, y, w, h, area))

        boxes.sort(key=lambda box: box[0])

        for x, y, w, h, area in boxes:

            center_x = x + w // 2
            center_y = y + h // 2

            color_status[color_name] = True

            detections.append({
                "color": color_name,
                "center": [center_x, center_y],
                "size": [w, h],
                "bbox": [x, y, w, h],
                "area": float(area)
            })

            cv2.rectangle(
                image,
                (x, y),
                (x + w, y + h),
                DRAW_COLORS[color_name],
                3
            )

            cv2.putText(
                image,
                color_name,
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                DRAW_COLORS[color_name],
                2
            )

    result = {
        "image_path": (
            str(image_path)
            if image_path else None
        ),
        "image_source": source,
        "image_width": image_width,
        "image_height": image_height,
        "coordinate_frame": "image_pixels",
        "status": (
            "DETECTED" if detections
            else "NOT_DETECTED"
        ),
        "detected_count": len(detections),
        "color_status": color_status,
        "detections": detections
    }

    return result, image
