import numpy as np

from config import (
    OBSTACLE_TRIGGER_DISTANCE,
    OBSTACLE_STOP_DISTANCE,
    LIDAR_FRONT_ANGLE,
    LIDAR_SIDE_ANGLE,
    DETOUR_LATERAL_DISTANCE,
    DETOUR_FORWARD_DISTANCE,
    DETOUR_WAYPOINT_TOLERANCE,
    DETOUR_MAX_SPEED,
    DETOUR_TURN_SPEED
)


class ObstacleAvoidance:

    def __init__(self):

        # None means normal navigation
        self.detour_index = None

        self.detour_targets = []

        # +1 = left, -1 = right
        self.turn_direction = 1.0


    def compute(
        self,
        angles,
        ranges,
        nominal_v,
        nominal_omega,
        estimated_state,
        waypoint_controller,
        goal_active=True,
        primary_goal=None
    ):

        # Stop after final navigation target
        if not goal_active:

            self.detour_index = None

            return (
                0.0, 0.0, False,
                None, None, None
            )

        # Do not move without LiDAR measurements
        if angles is None or ranges is None:

            return (
                0.0, 0.0, False,
                None, None, None
            )

        angles_deg = np.degrees(angles)

        front_mask = (
            np.abs(angles_deg)
            <= LIDAR_FRONT_ANGLE
        )

        left_mask = (
            (angles_deg > LIDAR_FRONT_ANGLE)
            &
            (angles_deg <= LIDAR_SIDE_ANGLE)
        )

        right_mask = (
            (angles_deg < -LIDAR_FRONT_ANGLE)
            &
            (angles_deg >= -LIDAR_SIDE_ANGLE)
        )

        front_distance = float(
            np.min(ranges[front_mask])
        )

        left_distance = float(
            np.min(ranges[left_mask])
        )

        right_distance = float(
            np.min(ranges[right_mask])
        )


        
# CHECK WHETHER DIRECT PATH TO ORIGINAL GOAL IS CLEAR


        if (
            self.detour_index is not None
            and primary_goal is not None
        ):

            x, y, theta = estimated_state

            goal_dx = primary_goal[0] - x
            goal_dy = primary_goal[1] - y

            goal_distance = np.hypot(
                goal_dx,
                goal_dy
            )

            # Direction from robot to original goal
            goal_direction = np.arctan2(
                goal_dy,
                goal_dx
            )

            # Convert goal direction into robot frame
            goal_bearing = (
                goal_direction - theta
            )

            goal_bearing = np.arctan2(
                np.sin(goal_bearing),
                np.cos(goal_bearing)
            )

            # Compare LiDAR ray directions with goal bearing
            angle_difference = (
                angles - goal_bearing
            )

            angle_difference = np.arctan2(
                np.sin(angle_difference),
                np.cos(angle_difference)
            )

            # Check a corridor wide enough for the robot
            corridor_half_angle = max(
                np.deg2rad(12.0),
                np.arctan2(
                    0.24,
                    max(goal_distance, 0.05)
                )
            )

            goal_mask = (
                np.abs(angle_difference)
                <= corridor_half_angle
            )

            if np.any(goal_mask):

                goal_path_clearance = float(
                    np.min(ranges[goal_mask])
                )

                # Return to primary goal only when the
                # entire corridor is clear, including a margin
                if (
                    self.detour_index >= 1
                    and
                    front_distance > OBSTACLE_TRIGGER_DISTANCE + 0.15
                    and
                    min(left_distance, right_distance) > 0.30
                    and
                    goal_path_clearance > goal_distance + 0.15
                ):

                    print(
                        "\nDirect path to original goal is clear."
                    )

                    print(
                        "Exiting detour mode."
                    )

                    self.detour_index = None
                    self.detour_targets = []

                    return (
                        nominal_v,
                        nominal_omega,
                        False,
                        front_distance,
                        left_distance,
                        right_distance
                    )

        
        # CREATE DYNAMIC DETOUR
        

        if self.detour_index is None:

            if front_distance < OBSTACLE_TRIGGER_DISTANCE:

                # Choose the more open side
                if left_distance >= right_distance:
                    self.turn_direction = 1.0
                else:
                    self.turn_direction = -1.0

                x, y, theta = estimated_state

                # Forward direction in world frame
                forward = np.array([
                    np.cos(theta),
                    np.sin(theta)
                ])

                # Left direction in world frame
                left = np.array([
                    -np.sin(theta),
                    np.cos(theta)
                ])

                current_position = np.array([x, y])

                # First move sideways away from obstacle
                detour_1 = (
                    current_position
                    + self.turn_direction
                    * DETOUR_LATERAL_DISTANCE
                    * left
                )

                # Then move beyond the obstacle
                detour_2 = (
                    detour_1
                    + DETOUR_FORWARD_DISTANCE
                    * forward
                )

                self.detour_targets = [
                    detour_1,
                    detour_2
                ]

                self.detour_index = 0

                print(
                    f"Trigger distances: "
                    f"front={front_distance:.3f}, "
                    f"left={left_distance:.3f}, "
                    f"right={right_distance:.3f}"
                )

                print("\nObstacle detected.")
                print("Dynamic detour created.")
                print("Detour 1:", detour_1)
                print("Detour 2:", detour_2)

        
        # FOLLOW DETOUR WAYPOINTS
        

        if self.detour_index is not None:

            target = self.detour_targets[
                self.detour_index
            ]

            (
                v_command,
                omega_command,
                reached,
                distance
            ) = waypoint_controller.compute(
                estimated_state,
                target
            )

            # Slightly larger tolerance for detour points
            reached = (
                distance <= DETOUR_WAYPOINT_TOLERANCE
            )

            if reached:

                print(
                    f"Detour point "
                    f"{self.detour_index + 1} reached."
                )

                self.detour_index += 1

                if self.detour_index >= len(
                    self.detour_targets
                ):

                    self.detour_index = None

                    print(
                        "Detour completed. "
                        "Resuming target navigation."
                    )

                    return (
                        nominal_v,
                        nominal_omega,
                        False,
                        front_distance,
                        left_distance,
                        right_distance
                    )

                target = self.detour_targets[
                    self.detour_index
                ]

                (
                    v_command,
                    omega_command,
                    _,
                    _
                ) = waypoint_controller.compute(
                    estimated_state,
                    target
                )

            v_command = min(
                v_command,
                DETOUR_MAX_SPEED
            )

            # Stop forward movement if obstacle is too close
            if front_distance < OBSTACLE_STOP_DISTANCE:

                v_command = 0.0

                omega_command = (
                    self.turn_direction
                    * DETOUR_TURN_SPEED
                )

            return (
                v_command,
                omega_command,
                True,
                front_distance,
                left_distance,
                right_distance
            )

        
        # NORMAL NAVIGATION
        

        return (
            nominal_v,
            nominal_omega,
            False,
            front_distance,
            left_distance,
            right_distance
        )