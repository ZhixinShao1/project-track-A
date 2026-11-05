import numpy as np


def wrap_angle(angle):
    """
    Wrap an angle to the range [-pi, pi].
    """
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def motion_model(state, control, dt):
    """
    Discrete nonlinear motion model for a differential-drive robot.

    Parameters
    ----------
    state : array-like
        [x, y, theta]

    control : array-like
        [v, omega]

    dt : float
        Time step in seconds.

    Returns
    -------
    numpy.ndarray
        Predicted state [x, y, theta].
    """

    x, y, theta = state
    v, omega = control

    x_next = x + v * np.cos(theta) * dt
    y_next = y + v * np.sin(theta) * dt
    theta_next = theta + omega * dt

    theta_next = wrap_angle(theta_next)

    return np.array([
        x_next,
        y_next,
        theta_next
    ])


def linearise_continuous(theta_ref, v_ref):
    """
    Linearise the nonlinear robot model around
    a reference heading and velocity.

    Returns
    -------
    A : numpy.ndarray
        State matrix.

    B : numpy.ndarray
        Input matrix.
    """

    A = np.array([
        [0.0, 0.0, -v_ref * np.sin(theta_ref)],
        [0.0, 0.0,  v_ref * np.cos(theta_ref)],
        [0.0, 0.0,  0.0]
    ])

    B = np.array([
        [np.cos(theta_ref), 0.0],
        [np.sin(theta_ref), 0.0],
        [0.0,               1.0]
    ])

    return A, B


def controllability_matrix(A, B):
    """
    Construct the controllability matrix:
    [B, AB, A^2B]
    """

    n = A.shape[0]

    blocks = [B]
    current = B

    for _ in range(1, n):
        current = A @ current
        blocks.append(current)

    return np.hstack(blocks)


def observability_matrix(A, C):
    """
    Construct the observability matrix:
    [C
     CA
     CA^2]
    """

    n = A.shape[0]

    blocks = [C]
    current = C

    for _ in range(1, n):
        current = current @ A
        blocks.append(current)

    return np.vstack(blocks)