from pathlib import Path

import cv2
import numpy as np


# ArUco dictionary
dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)

# Marker settings
MARKER_ID = 0
MARKER_SIZE_PX = 600
CANVAS_SIZE = 800

# Generate marker
marker = cv2.aruco.generateImageMarker(
    dictionary,
    MARKER_ID,
    MARKER_SIZE_PX
)

# Add white margin around the marker
canvas = np.ones(
    (CANVAS_SIZE, CANVAS_SIZE),
    dtype=np.uint8
) * 255

margin = (CANVAS_SIZE - MARKER_SIZE_PX) // 2

canvas[
    margin:margin + MARKER_SIZE_PX,
    margin:margin + MARKER_SIZE_PX
] = marker

# Output path
output_path = (
    Path(__file__).parent.parent
    / "models"
    / "turtlebot3"
    / "assets"
    / "aruco_marker_0.png"
)

cv2.imwrite(
    str(output_path),
    canvas
)

print(
    f"ArUco marker saved to: {output_path}"
)