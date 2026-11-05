import numpy as np

from config import (
    ARUCO_WORLD_X,
    ARUCO_WORLD_Y,
    DOCK_TARGET_BASE_RANGE,
    DOCK_RANGE_TOLERANCE,
    DOCK_BEARING_TOLERANCE_DEG,
    DOCK_MOVE_BEARING_LIMIT_DEG,
    DOCK_DISTANCE_GAIN,
    DOCK_HEADING_GAIN,
    DOCK_MAX_LINEAR_SPEED,
    DOCK_MAX_ANGULAR_SPEED,
    DOCK_CAMERA_EKF_MAX_DIFFERENCE
)


class VisualDockingController:

    def __init__(self):
        self.completed = False
        self.search_direction = 1.0

    def compute(
        self,
        estimated_state,
        base_range,
        base_bearing
    ):

        if self.completed:
            return 0.0, 0.0, True, "COMPLETE"

        state = np.asarray(
            estimated_state,
            dtype=float
        )

        if not np.all(np.isfinite(state)):
            return 0.0, 0.0, False, "INVALID_EKF"

        x, y, theta = state

        # Direction to the station using EKF state
        dx = ARUCO_WORLD_X - x
        dy = ARUCO_WORLD_Y - y

        estimated_distance = np.hypot(dx, dy)

        desired_heading = np.arctan2(dy, dx)

        heading_error = np.arctan2(
            np.sin(desired_heading - theta),
            np.cos(desired_heading - theta)
        )

        
        # CAMERA NOT AVAILABLE
        

        if (
            base_range is None
            or base_bearing is None
            or not np.isfinite(base_range)
            or not np.isfinite(base_bearing)
        ):

            # Stop driving. Use EKF heading to look
            # toward the known station position.
                # First face the estimated charging station direction
            if abs(heading_error) > np.deg2rad(20.0):

                omega = np.clip(
                    DOCK_HEADING_GAIN * heading_error,
                    -DOCK_MAX_ANGULAR_SPEED,
                    DOCK_MAX_ANGULAR_SPEED
                )

                return (
                    0.0,
                    float(omega),
                    False,
                    "SEARCHING_ARUCO"
                )

            # Relative orientation to estimated station direction
            relative_angle = np.arctan2(
                np.sin(theta - desired_heading),
                np.cos(theta - desired_heading)
            )

            scan_limit = np.deg2rad(18.0)

            # Reverse search direction at angular limits
            if relative_angle >= scan_limit:

                self.search_direction = -1.0

            elif relative_angle <= -scan_limit:

                self.search_direction = 1.0

            # Rotate slowly while keeping linear speed zero
            return (
                0.0,
                0.15 * self.search_direction,
                False,
                "SCANNING_ARUCO"
            )

        
        # CAMERA / EKF CONSISTENCY CHECK
        

        if (
            abs(estimated_distance - base_range)
            > DOCK_CAMERA_EKF_MAX_DIFFERENCE
        ):
            return (
                0.0,
                0.0,
                False,
                "CAMERA_EKF_MISMATCH"
            )

        
        # CHECK DOCKING COMPLETION
        

        bearing_tolerance = np.deg2rad(
            DOCK_BEARING_TOLERANCE_DEG
        )

        if (
            base_range
            <= DOCK_TARGET_BASE_RANGE
            + DOCK_RANGE_TOLERANCE
            and
            abs(base_bearing) <= bearing_tolerance
        ):

            self.completed = True

            return (
                0.0,
                0.0,
                True,
                "COMPLETE"
            )

        
        # VISUAL FEEDBACK CONTROLLER
       

        omega_command = np.clip(
            DOCK_HEADING_GAIN * base_bearing,
            -DOCK_MAX_ANGULAR_SPEED,
            DOCK_MAX_ANGULAR_SPEED
        )

        distance_error = (
            base_range - DOCK_TARGET_BASE_RANGE
        )

        # Do not move forward until reasonably aligned
        if (
            abs(base_bearing)
            > np.deg2rad(
                DOCK_MOVE_BEARING_LIMIT_DEG
            )
        ):

            v_command = 0.0
            status = "ALIGNING"

        else:

            v_command = np.clip(
                DOCK_DISTANCE_GAIN
                * max(distance_error, 0.0),
                0.0,
                DOCK_MAX_LINEAR_SPEED
            )

            status = "APPROACHING"

        return (
            float(v_command),
            float(omega_command),
            False,
            status
        )