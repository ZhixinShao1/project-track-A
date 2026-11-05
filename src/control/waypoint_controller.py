import numpy as np

from src.model.state_space import wrap_angle

from config import (
    WAYPOINT_TOLERANCE,
    DISTANCE_GAIN,
    HEADING_GAIN,
    MAX_LINEAR_VELOCITY,
    MAX_ANGULAR_VELOCITY
)


class WaypointController:

    def __init__(self):

        self.distance_gain = DISTANCE_GAIN
        self.heading_gain = HEADING_GAIN

        self.max_linear_velocity = (
            MAX_LINEAR_VELOCITY
        )

        self.max_angular_velocity = (
            MAX_ANGULAR_VELOCITY
        )

        self.tolerance = WAYPOINT_TOLERANCE


    def compute(
        self,
        estimated_state,
        waypoint
    ):
        """
        Compute linear and angular velocity commands
        using the EKF estimated state only.
        """

        x = estimated_state[0]
        y = estimated_state[1]
        theta = estimated_state[2]

        waypoint_x = waypoint[0]
        waypoint_y = waypoint[1]

        dx = waypoint_x - x
        dy = waypoint_y - y

        distance = np.sqrt(
            dx ** 2 + dy ** 2
        )

        # Check whether waypoint has been reached
        if distance <= self.tolerance:

            return (
                0.0,
                0.0,
                True,
                distance
            )

        desired_heading = np.arctan2(
            dy,
            dx
        )

        heading_error = wrap_angle(
            desired_heading - theta
        )

        # State-feedback control
        v_command = (
            self.distance_gain
            * distance
        )

        omega_command = (
            self.heading_gain
            * heading_error
        )

        # Slow down when robot is not pointing
        # toward the waypoint
        v_command *= max(
            0.0,
            np.cos(heading_error)
        )

        # Apply velocity limits
        v_command = np.clip(
            v_command,
            0.0,
            self.max_linear_velocity
        )

        omega_command = np.clip(
            omega_command,
            -self.max_angular_velocity,
            self.max_angular_velocity
        )

        return (
            v_command,
            omega_command,
            False,
            distance
        )