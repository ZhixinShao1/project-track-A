import matplotlib.pyplot as plt


def plot_trajectory(
    true_x,
    true_y,
    odom_x,
    odom_y,
    ekf_x,
    ekf_y
):
    plt.figure()

    plt.plot(
        true_x,
        true_y,
        label="Ground Truth"
    )

    plt.plot(
        odom_x,
        odom_y,
        label="Wheel Odometry"
    )

    plt.plot(
        ekf_x,
        ekf_y,
        label="EKF Estimate"
    )

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")

    plt.title(
        "Robot State Estimation Comparison"
    )

    plt.axis("equal")
    plt.grid(True)
    plt.legend()

    plt.show()

def plot_ekf_uncertainty(
    time_history,
    true_x,
    ekf_x,
    true_theta,
    ekf_theta,
    sigma_x,
    sigma_theta
):
    import numpy as np
    import matplotlib.pyplot as plt

    time_history = np.array(time_history)

    true_x = np.array(true_x)
    ekf_x = np.array(ekf_x)

    true_theta = np.array(true_theta)
    ekf_theta = np.array(ekf_theta)

    sigma_x = np.array(sigma_x)
    sigma_theta = np.array(sigma_theta)

    # X estimation error
    x_error = true_x - ekf_x

    plt.figure()

    plt.plot(
        time_history,
        x_error,
        label="X Estimation Error"
    )

    plt.plot(
        time_history,
        2.0 * sigma_x,
        "--",
        label="+2 Sigma"
    )

    plt.plot(
        time_history,
        -2.0 * sigma_x,
        "--",
        label="-2 Sigma"
    )

    plt.xlabel("Time [s]")
    plt.ylabel("X Error [m]")

    plt.title(
        "EKF Position Error and Uncertainty"
    )

    plt.grid(True)
    plt.legend()


    # Heading estimation error
    theta_error = true_theta - ekf_theta

    theta_error = (
        theta_error + np.pi
    ) % (
        2.0 * np.pi
    ) - np.pi

    plt.figure()

    plt.plot(
        time_history,
        theta_error,
        label="Heading Estimation Error"
    )

    plt.plot(
        time_history,
        2.0 * sigma_theta,
        "--",
        label="+2 Sigma"
    )

    plt.plot(
        time_history,
        -2.0 * sigma_theta,
        "--",
        label="-2 Sigma"
    )

    plt.xlabel("Time [s]")
    plt.ylabel("Heading Error [rad]")

    plt.title(
        "EKF Heading Error and Uncertainty"
    )

    plt.grid(True)
    plt.legend()

    plt.show()