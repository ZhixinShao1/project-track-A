import numpy as np

from src.model.state_space import motion_model


class Odometry:
    def __init__(self, initial_state=None):

        if initial_state is None:
            initial_state = np.zeros(3)

        self.state = np.array(
            initial_state,
            dtype=float
        )


    def update(self, v, omega, dt):
        """
        Update odometry using measured
        linear and angular velocity.
        """

        control = np.array([
            v,
            omega
        ])

        self.state = motion_model(
            self.state,
            control,
            dt
        )

        return self.state.copy()


    def reset(self, state=None):

        if state is None:
            state = np.zeros(3)

        self.state = np.array(
            state,
            dtype=float
        )