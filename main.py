from src.sensors.wheel_encoder import WheelEncoder
from src.estimation.odometry import Odometry
from src.control.differential_drive import velocity_to_wheels
from src.utils.transforms import quaternion_to_yaw
from src.utils.plotting import (
    plot_trajectory,
    plot_ekf_uncertainty
)
from src.perception.aruco_detector import ArucoDetector
from config import (
    CAMERA_WIDTH,
    CAMERA_HEIGHT,
    CAMERA_FOVY,
    ARUCO_MARKER_ID,
    ARUCO_MARKER_SIZE,
    ARUCO_WORLD_X,
    ARUCO_WORLD_Y,
    EKF_INITIAL_POSITION_STD,
    EKF_INITIAL_HEADING_STD,
    EKF_PROCESS_X_STD,
    EKF_PROCESS_Y_STD,
    EKF_PROCESS_THETA_STD,
    EKF_RANGE_STD,
    EKF_BEARING_STD,
    CAMERA_OFFSET_X,
    CAMERA_OFFSET_Y,
    WAYPOINTS
)
from src.estimation.ekf import ExtendedKalmanFilter
import time
from pathlib import Path

import mujoco
import mujoco.viewer

import cv2
import numpy as np
from src.control.waypoint_controller import WaypointController


MODEL_PATH = (
    Path(__file__).parent
    / "models"
    / "turtlebot3"
    / "scene_turtlebot3_waffle_pi.xml"
)


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    renderer = mujoco.Renderer(
    model,
    height=CAMERA_HEIGHT,
    width=CAMERA_WIDTH
    )


# CAMERA INTRINSIC MATRIX

    fovy_rad = np.deg2rad(
        CAMERA_FOVY
    )

    fy = CAMERA_HEIGHT / (
        2.0 * np.tan(
            fovy_rad / 2.0
        )
    )

    fx = fy

    cx = (
        CAMERA_WIDTH - 1
    ) / 2.0

    cy = (
        CAMERA_HEIGHT - 1
    ) / 2.0

    camera_matrix = np.array(
        [
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0]
        ],
        dtype=np.float64
    )

    distortion_coefficients = np.zeros(
        (5, 1),
        dtype=np.float64
    )

    print(
        "Camera matrix:"
    )

    print(
        camera_matrix
    )


    aruco_detector = ArucoDetector(
        camera_matrix=camera_matrix,
        distortion_coefficients=distortion_coefficients,
        marker_size=ARUCO_MARKER_SIZE,
        target_id=ARUCO_MARKER_ID
    )



# GROUND TRUTH - FOR EVALUATION ONLY


    base_joint_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        "base_joint"
    )

    base_qpos = model.jnt_qposadr[
        base_joint_id
    ]




    encoder = WheelEncoder(
        model,
        noise_std_left=0.03,
        noise_std_right=0.03
    )

    odometry = Odometry(
        initial_state=[0.0, 0.0, 0.0]
    )


    initial_covariance = np.diag(
        [
            EKF_INITIAL_POSITION_STD ** 2,
            EKF_INITIAL_POSITION_STD ** 2,
            EKF_INITIAL_HEADING_STD ** 2
        ]
    )

    process_noise = np.diag(
        [
            EKF_PROCESS_X_STD ** 2,
            EKF_PROCESS_Y_STD ** 2,
            EKF_PROCESS_THETA_STD ** 2
        ]
    )

    measurement_noise = np.diag(
        [
            EKF_RANGE_STD ** 2,
            EKF_BEARING_STD ** 2
        ]
    )

    ekf = ExtendedKalmanFilter(
        initial_state=[
            0.0,
            0.0,
            0.0
        ],
        initial_covariance=initial_covariance,
        process_noise=process_noise,
        measurement_noise=measurement_noise,
        landmark_position=[
            ARUCO_WORLD_X,
            ARUCO_WORLD_Y
        ]
    )

    waypoint_controller = WaypointController()

    waypoints = np.array(
        WAYPOINTS,
        dtype=float
    )

    current_waypoint_index = 0

    all_waypoints_reached = False







    print("MuJoCo model loaded successfully.")

    left_motor_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        "wheel_left"
    )

    right_motor_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        "wheel_right"
    )

    print("Left motor ID:", left_motor_id)
    print("Right motor ID:", right_motor_id)

    last_print_time = -1
    last_camera_time = -1.0
    last_marker_state = False

    time_history = []

    true_x_history = []
    true_y_history = []

    odom_x_history = []
    odom_y_history = []

    ekf_x_history = []
    ekf_y_history = []

    true_theta_history = []
    ekf_theta_history = []

    sigma_x_history = []
    sigma_y_history = []
    sigma_theta_history = []

    ekf_state = ekf.state.copy()

    with mujoco.viewer.launch_passive(model, data) as viewer:

        while viewer.is_running():

            if not all_waypoints_reached:

                target_waypoint = waypoints[
                    current_waypoint_index
                ]

                (
                    v_command,
                    omega_command,
                    waypoint_reached,
                    waypoint_distance
                ) = waypoint_controller.compute(
                    ekf_state,
                    target_waypoint
                )

            else:

                v_command = 0.0
                omega_command = 0.0

                waypoint_reached = True
                waypoint_distance = 0.0


            if (
                waypoint_reached
                and
                not all_waypoints_reached
            ):

                print(
                    f"\nWaypoint "
                    f"{current_waypoint_index + 1} reached."
                )

                current_waypoint_index += 1

                if current_waypoint_index >= len(
                    waypoints
                ):

                    all_waypoints_reached = True

                    print(
                        "\nAll waypoints reached."
                    )


            



            omega_left, omega_right = velocity_to_wheels(
                v_command,
                omega_command
            )

            data.ctrl[left_motor_id] = omega_left
            data.ctrl[right_motor_id] = omega_right
            
            
            
           

            mujoco.mj_step(model, data)


 # Read wheel encoder
            omega_left_meas, omega_right_meas = encoder.read(
                data
            )

# Convert wheel measurements to body velocity
            v_enc, omega_enc = encoder.body_velocity(
                omega_left_meas,
                omega_right_meas
            )

# Update wheel odometry
            odom_state = odometry.update(
                v_enc,
                omega_enc,
                model.opt.timestep
            )

            
# EKF PREDICTION


            ekf.predict(
                v_enc,
                omega_enc,
                model.opt.timestep
            )



           
# FRONT CAMERA


            if data.time - last_camera_time >= 0.05:

                renderer.update_scene(
                    data,
                    camera="front_camera"
                )

                rgb_image = renderer.render()

                bgr_image = cv2.cvtColor(
                    rgb_image,
                    cv2.COLOR_RGB2BGR
                )

    # ArUco detection
                (
                    camera_display,
                    marker_detected,
                    marker_corners,
                    marker_range,
                    marker_bearing
                ) = aruco_detector.detect(
                    bgr_image
                )



# CAMERA FRAME  ROBOT BASE FRAME


                base_range = None
                base_bearing = None

                if (
                    marker_detected
                    and
                    marker_range is not None
                    and
                    marker_bearing is not None
                    and
                    np.isfinite(marker_range)
                    and
                    np.isfinite(marker_bearing)
                ):

                    # Landmark position relative to camera
                    landmark_forward_camera = (
                        marker_range
                        * np.cos(marker_bearing)
                    )

                    landmark_left_camera = (
                        marker_range
                        * np.sin(marker_bearing)
                    )

                    # Convert to robot base frame
                    landmark_forward_base = (
                        CAMERA_OFFSET_X
                        + landmark_forward_camera
                    )

                    landmark_left_base = (
                        CAMERA_OFFSET_Y
                        + landmark_left_camera
                    )

                    base_range = np.sqrt(
                        landmark_forward_base ** 2
                        +
                        landmark_left_base ** 2
                    )

                    base_bearing = np.arctan2(
                        landmark_left_base,
                        landmark_forward_base
                    )


# EKF camera update
                if (
                    marker_detected
                    and
                    base_range is not None
                    and
                    base_bearing is not None
                    and
                    np.isfinite(base_range)
                    and
                    np.isfinite(base_bearing)
                ):

                    ekf.update_range_bearing(
                        base_range,
                        base_bearing
                    )

                ekf_state = ekf.state.copy()


    # Print only when detection state changes
                if marker_detected != last_marker_state:

                    if marker_detected:
                        print("ArUco marker detected.")

                    else:
                        print("ArUco marker lost.")

                    last_marker_state = marker_detected

                cv2.imshow(
                    "TurtleBot Front Camera",
                    camera_display
                )


                


                cv2.waitKey(1)

                last_camera_time = data.time


           
# GROUND TRUTH - EVALUATION ONLY

            x_true = data.qpos[
                base_qpos
            ]

            y_true = data.qpos[
                base_qpos + 1
            ]

            qw = data.qpos[
                base_qpos + 3
            ]

            qx = data.qpos[
                base_qpos + 4
            ]

            qy = data.qpos[
                base_qpos + 5
            ]

            qz = data.qpos[
                base_qpos + 6
            ]

            theta_true = quaternion_to_yaw(
                qw,
                qx,
                qy,
                qz
            )

            time_history.append(
                data.time
            )

            true_x_history.append(
                x_true
            )

            true_y_history.append(
                y_true
            )

            odom_x_history.append(
                odom_state[0]
            )

            odom_y_history.append(
                odom_state[1]
            )

            ekf_x_history.append(
                ekf_state[0]
            )

            ekf_y_history.append(
                ekf_state[1]
            )

            true_theta_history.append(
                theta_true
            )

            ekf_theta_history.append(
                ekf_state[2]
            )

            sigma_x_history.append(
                np.sqrt(ekf.P[0, 0])
            )

            sigma_y_history.append(
                np.sqrt(ekf.P[1, 1])
            )

            sigma_theta_history.append(
                np.sqrt(ekf.P[2, 2])
            )
            current_second = int(data.time)


            if current_second != last_print_time:

                print(
                    f"\nTime: {data.time:.2f}"
                )

                print(
                    f"TRUE: "
                    f"x={x_true:.3f}, "
                    f"y={y_true:.3f}, "
                    f"theta={theta_true:.3f}"
                )

                print(
                    f"ODOM: "
                    f"x={odom_state[0]:.3f}, "
                    f"y={odom_state[1]:.3f}, "
                    f"theta={odom_state[2]:.3f}"
                )

                print(
                    f"EKF:  "
                    f"x={ekf_state[0]:.3f}, "
                    f"y={ekf_state[1]:.3f}, "
                    f"theta={ekf_state[2]:.3f}"
                )

                if not all_waypoints_reached:

                    print(
                        f"CONTROL: "
                        f"waypoint="
                        f"{current_waypoint_index + 1}/"
                        f"{len(waypoints)}, "
                        f"distance={waypoint_distance:.3f} m, "
                        f"v={v_command:.3f} m/s, "
                        f"omega={omega_command:.3f} rad/s"
                    )

                else:

                    print(
                        "CONTROL: "
                        "All waypoints completed."
                    )

                if (
                    marker_detected
                    and marker_range is not None
                    and marker_bearing is not None
                    and base_range is not None
                    and base_bearing is not None
                ):
                    print(
                        f"CAMERA: "
                        f"range={marker_range:.3f}, "
                        f"bearing={np.degrees(marker_bearing):.2f} deg"
                    )

                    print(
                        f"BASE:   "
                        f"range={base_range:.3f}, "
                        f"bearing={np.degrees(base_bearing):.2f} deg"
                    )

                else:
                    print(
                        "CAMERA: ArUco marker not visible"
                    )

                
                last_print_time = current_second
                



            viewer.sync()

            time.sleep(model.opt.timestep)


    renderer.close()
    cv2.destroyAllWindows()


    plot_trajectory(
    true_x_history,
    true_y_history,
    odom_x_history,
    odom_y_history,
    ekf_x_history,
    ekf_y_history
    )

    plot_ekf_uncertainty(
    time_history,
    true_x_history,
    ekf_x_history,
    true_theta_history,
    ekf_theta_history,
    sigma_x_history,
    sigma_theta_history
    )


if __name__ == "__main__":
    main()
