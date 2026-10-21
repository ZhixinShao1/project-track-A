import matplotlib.pyplot as plt


def plot_trajectory(
    true_x,
    true_y,
    odom_x,
    odom_y
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

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")

    plt.title(
        "Ground Truth vs Wheel Odometry"
    )

    plt.axis("equal")
    plt.grid(True)
    plt.legend()

    plt.show()