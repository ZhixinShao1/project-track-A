import numpy as np


def quaternion_to_yaw(qw, qx, qy, qz):
    siny_cosp = 2.0 * (
        qw * qz + qx * qy
    )

    cosy_cosp = 1.0 - 2.0 * (
        qy * qy + qz * qz
    )

    yaw = np.arctan2(
        siny_cosp,
        cosy_cosp
    )

    return yaw