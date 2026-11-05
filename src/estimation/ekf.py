import numpy as np

from src.model.state_space import (
    motion_model,
    wrap_angle
)


class ExtendedKalmanFilter:

    def __init__(
        self,
        initial_state,
        initial_covariance,
        process_noise,
        measurement_noise,
        landmark_position
    ):

        self.state = np.array(
            initial_state,
            dtype=float
        )

        self.P = np.array(
            initial_covariance,
            dtype=float
        )

        self.Q = np.array(
            process_noise,
            dtype=float
        )

        self.R = np.array(
            measurement_noise,
            dtype=float
        )

        self.landmark_position = np.array(
            landmark_position,
            dtype=float
        )


    def predict(
        self,
        v,
        omega,
        dt
    ):

        theta = self.state[2]

        # Jacobian of motion model
        F = np.array(
            [
                [
                    1.0,
                    0.0,
                    -v * np.sin(theta) * dt
                ],
                [
                    0.0,
                    1.0,
                    v * np.cos(theta) * dt
                ],
                [
                    0.0,
                    0.0,
                    1.0
                ]
            ]
        )

        control = np.array(
            [v, omega]
        )

        # State prediction
        self.state = motion_model(
            self.state,
            control,
            dt
        )

        # Covariance prediction
        self.P = (
            F
            @ self.P
            @ F.T
            + self.Q * dt
        )

        self.state[2] = wrap_angle(
            self.state[2]
        )

        return self.state.copy()


    def update_range_bearing(
        self,
        measured_range,
        measured_bearing
    ):

        if (
            not np.isfinite(measured_range)
            or
            not np.isfinite(measured_bearing)
            or
            measured_range <= 0.0
        ):
            return self.state.copy()

        x = self.state[0]
        y = self.state[1]
        theta = self.state[2]

        landmark_x = self.landmark_position[0]
        landmark_y = self.landmark_position[1]

        dx = landmark_x - x
        dy = landmark_y - y

        q = dx ** 2 + dy ** 2

        if q < 1e-9:
            return self.state.copy()

        predicted_range = np.sqrt(q)

        predicted_bearing = wrap_angle(
            np.arctan2(
                dy,
                dx
            )
            - theta
        )

        predicted_measurement = np.array(
            [
                predicted_range,
                predicted_bearing
            ]
        )

        measurement = np.array(
            [
                measured_range,
                measured_bearing
            ]
        )

        innovation = (
            measurement
            - predicted_measurement
        )

        innovation[1] = wrap_angle(
            innovation[1]
        )

        # Measurement Jacobian
        H = np.array(
            [
                [
                    -dx / predicted_range,
                    -dy / predicted_range,
                    0.0
                ],
                [
                    dy / q,
                    -dx / q,
                    -1.0
                ]
            ]
        )

        S = (
            H
            @ self.P
            @ H.T
            + self.R
        )

        K = (
            self.P
            @ H.T
            @ np.linalg.inv(S)
        )

        self.state = (
            self.state
            + K @ innovation
        )

        self.state[2] = wrap_angle(
            self.state[2]
        )

        I = np.eye(3)

        self.P = (
            I - K @ H
        ) @ self.P

        return self.state.copy()