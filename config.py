# TurtleBot3 Waffle Pi parameters

WHEEL_RADIUS = 0.033
WHEEL_BASE = 0.287

# Simulation
CONTROL_DT = 0.01

# Camera
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
CAMERA_FOVY = 60.0

# Camera position relative to robot base
CAMERA_OFFSET_X = 0.10
CAMERA_OFFSET_Y = 0.0

# ArUco marker
ARUCO_MARKER_ID = 0

# Actual black ArUco square size
# The full textured board is 0.24 m,
# while the black marker occupies 600/800 of it.
ARUCO_MARKER_SIZE = 0.18


# EKF


# ArUco marker world position
ARUCO_WORLD_X = 1.0
ARUCO_WORLD_Y = 0.0

# Initial EKF uncertainty
EKF_INITIAL_POSITION_STD = 0.02
EKF_INITIAL_HEADING_STD = 0.035

# Process noise
EKF_PROCESS_X_STD = 0.01
EKF_PROCESS_Y_STD = 0.01
EKF_PROCESS_THETA_STD = 0.02

# Camera measurement noise
EKF_RANGE_STD = 0.02
EKF_BEARING_STD = 0.035

# WAYPOINT CONTROLLER

WAYPOINTS = [
    [0.30, 0.00],
    [0.30, 0.25],
    [0.60, 0.25],
    [0.75, 0.00]
]

WAYPOINT_TOLERANCE = 0.03

DISTANCE_GAIN = 0.8
HEADING_GAIN = 1.5

MAX_LINEAR_VELOCITY = 0.05
MAX_ANGULAR_VELOCITY = 0.8