import cv2
import numpy as np

from config import LIDAR_MAX_RANGE


def draw_lidar_scan(
    angles,
    ranges,
    image_size=400
):

    image = np.zeros(
        (
            image_size,
            image_size,
            3
        ),
        dtype=np.uint8
    )

    center = image_size // 2

    scale = (
        image_size * 0.45
        / LIDAR_MAX_RANGE
    )

    # Robot
    cv2.circle(
        image,
        (center, center),
        5,
        (255, 255, 255),
        -1
    )

    for angle, distance in zip(
        angles,
        ranges
    ):

        if distance >= LIDAR_MAX_RANGE:
            continue

        x = (
            center
            +
            int(
                distance
                * np.cos(angle)
                * scale
            )
        )

        y = (
            center
            -
            int(
                distance
                * np.sin(angle)
                * scale
            )
        )

        cv2.circle(
            image,
            (x, y),
            2,
            (0, 255, 0),
            -1
        )

    nearest_range = np.min(
        ranges
    )

    cv2.putText(
        image,
        f"Nearest: {nearest_range:.2f} m",
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2
    )

    cv2.putText(
        image,
        "2D LiDAR",
        (10, image_size - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2
    )

    return image