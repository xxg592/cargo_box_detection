import cv2
import numpy as np

# 创建一张浅灰色背景图片
image = np.full((720, 1280, 3), 245, dtype=np.uint8)

# 定义三个货物箱
boxes = [
    ("RED", (0, 0, 255), (120, 180, 360, 520)),
    ("GREEN", (0, 200, 0), (460, 180, 700, 520)),
    ("BLUE", (255, 0, 0), (800, 180, 1040, 520))
]

# 绘制货物箱
for name, color, (x1, y1, x2, y2) in boxes:
    cv2.rectangle(image, (x1, y1), (x2, y2), color, -1)

    cv2.rectangle(
        image,
        (x1, y1),
        (x2, y2),
        (40, 40, 40),
        3
    )

    cv2.putText(
        image,
        name,
        (x1 + 20, y1 + 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.2,
        (255, 255, 255),
        3
    )

# 保存图片
cv2.imwrite("images/test_boxes.png", image)

print("Test image created successfully!")
print("Image width: 1280")
print("Image height: 720")
print("Cargo boxes: RED, GREEN, BLUE")
