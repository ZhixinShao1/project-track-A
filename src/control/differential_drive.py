from config import WHEEL_RADIUS, WHEEL_BASE


def velocity_to_wheels(v, omega):
    """
    Convert robot linear and angular velocity
    to left and right wheel angular velocities.

    Parameters
    ----------
    v : float
        Linear velocity in m/s.

    omega : float
        Angular velocity in rad/s.

    Returns
    -------
    omega_left : float
        Left wheel angular velocity in rad/s.

    omega_right : float
        Right wheel angular velocity in rad/s.
    """

    omega_left = (
        v - (WHEEL_BASE / 2.0) * omega
    ) / WHEEL_RADIUS

    omega_right = (
        v + (WHEEL_BASE / 2.0) * omega
    ) / WHEEL_RADIUS

    return omega_left, omega_right