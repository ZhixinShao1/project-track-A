import numpy as np

from config import WHEEL_RADIUS, WHEEL_BASE


class WheelEncoder:
    def __init__(
        self,
        model,
        left_joint_name="wheel_left",
        right_joint_name="wheel_right",
        noise_std_left=0.0,
        noise_std_right=0.0
    ):
        import mujoco

        self.model = model

        self.left_joint_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            left_joint_name
        )

        self.right_joint_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            right_joint_name
        )

        self.left_dof = model.jnt_dofadr[
            self.left_joint_id
        ]

        self.right_dof = model.jnt_dofadr[
            self.right_joint_id
        ]

        self.noise_std_left = noise_std_left
        self.noise_std_right = noise_std_right


    def read(self, data):
        """
        Read wheel angular velocities.

        Returns
        -------
        omega_left : float
            Left wheel angular velocity [rad/s]

        omega_right : float
            Right wheel angular velocity [rad/s]
        """

        omega_left = data.qvel[self.left_dof]
        omega_right = data.qvel[self.right_dof]

        omega_left += np.random.normal(
            0.0,
            self.noise_std_left
        )

        omega_right += np.random.normal(
            0.0,
            self.noise_std_right
        )

        return omega_left, omega_right


    def body_velocity(self, omega_left, omega_right):
        """
        Convert wheel velocities to robot linear
        and angular velocity.
        """

        v = (
            WHEEL_RADIUS / 2.0
        ) * (
            omega_right + omega_left
        )

        omega = (
            WHEEL_RADIUS / WHEEL_BASE
        ) * (
            omega_right - omega_left
        )

        return v, omega