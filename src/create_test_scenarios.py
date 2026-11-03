import cv2
import numpy as np
from pathlib import Path

Path("images").mkdir(parents=True, exist_ok=True)

# Define cargo box colors in BGR format
colors = {
    "RED": (0, 0, 255),
    "GREEN": (0, 200, 0),
    "BLUE": (255, 0, 0),
    "YELLOW": (0, 255, 255)
}


def create_background():
    return np.full((720, 1280, 3), 245, dtype=np.uint8)


def draw_box(image, name, color, x1, y1, x2, y2):
    cv2.rectangle(image, (x1, y1), (x2, y2), color, -1)
    cv2.rectangle(image, (x1, y1), (x2, y2), (40, 40, 40), 3)

    cv2.putText(
        image,
        name,
        (x1 + 10, y1 + 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2
    )


# Scenario A: Different box positions
image = create_background()

draw_box(image, "RED", colors["RED"], 70, 80, 310, 320)
draw_box(image, "GREEN", colors["GREEN"], 480, 330, 720, 570)
draw_box(image, "BLUE", colors["BLUE"], 930, 100, 1170, 340)

cv2.imwrite("images/test_positions.png", image)


# Scenario B: Different box sizes
image = create_background()

draw_box(image, "RED", colors["RED"], 60, 180, 160, 310)
draw_box(image, "GREEN", colors["GREEN"], 420, 200, 620, 430)
draw_box(image, "BLUE", colors["BLUE"], 850, 120, 1190, 550)

cv2.imwrite("images/test_sizes.png", image)


# Scenario C: Empty scene
image = create_background()

cv2.line(
    image,
    (0, 550),
    (1280, 550),
    (200, 200, 200),
    3
)

cv2.imwrite("images/test_empty.png", image)

# Scenario D: all four supported cargo colours
image = create_background()

draw_box(image, "RED", colors["RED"], 60, 160, 340, 560)
draw_box(image, "GREEN", colors["GREEN"], 380, 160, 660, 560)
draw_box(image, "BLUE", colors["BLUE"], 700, 160, 980, 560)
draw_box(image, "YELLOW", colors["YELLOW"], 1020, 160, 1240, 560)

cv2.imwrite("images/test_four_colors.png", image)

print("Four test scenarios created successfully!")
print("1. test_positions.png")
print("2. test_sizes.png")
print("3. test_empty.png")
print("4. test_four_colors.png")
