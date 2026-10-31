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