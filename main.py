from src.sensors.wheel_encoder import WheelEncoder
from src.estimation.odometry import Odometry
from src.control.differential_drive import velocity_to_wheels
from src.utils.transforms import quaternion_to_yaw
from src.utils.plotting import (
    plot_trajectory,
    plot_ekf_uncertainty
)
from src.utils.evaluation import save_trial_results
import os
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
    WAYPOINTS,
    LIDAR_UPDATE_INTERVAL,
    CAMERA_RANGE_NOISE_STD,
    CAMERA_BEARING_NOISE_STD_DEG
)
from src.estimation.ekf import ExtendedKalmanFilter
import time
from pathlib import Path

import mujoco
import mujoco.viewer

import cv2
import numpy as np
from src.control.waypoint_controller import WaypointController
from src.sensors.lidar import Lidar2D
from src.utils.lidar_visualization import draw_lidar_scan
from src.control.obstacle_avoidance import ObstacleAvoidance
from src.control.visual_docking import VisualDockingController


MODEL_PATH = (
    Path(__file__).parent
    / "models"
    / "turtlebot3"
    / "scene_turtlebot3_waffle_pi.xml"
)


def main():
    run_seed = int(
        os.getenv("RUN_SEED", "2026")
    )

    np.random.seed(run_seed)

    model = mujoco.MjModel.from_xml_path(
        str(MODEL_PATH)
    )
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

    # Robot root body
    robot_root_body_id = int(
        model.jnt_bodyid[base_joint_id]
    )

    # Test obstacle
    obstacle_geom_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_GEOM,
        "obstacle_1"
    )

    if obstacle_geom_id == -1:
        raise ValueError(
            "obstacle_1 geom not found."
        )

    collision_detected = False




    encoder = WheelEncoder(
        model,
        noise_std_left=0.03,
        noise_std_right=0.03
    )

    lidar = Lidar2D(
        model,
        site_name="lidar_site"
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
    obstacle_avoidance = ObstacleAvoidance()

    visual_docking = VisualDockingController()

    docking_complete = False
    docking_status = "NAVIGATION"

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
    last_lidar_time = -1.0

    marker_detected = False
    marker_range = None
    marker_bearing = None

    base_range = None
    base_bearing = None

    lidar_angles = None
    lidar_ranges = None
    last_marker_state = False

    time_history = []

    v_history = []
    omega_history = []
    camera_visible_history = []

    finish_sample_count = None

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

    def is_robot_body(body_id):

        body_id = int(body_id)

        while body_id != 0:

            if body_id == robot_root_body_id:
                return True

            body_id = int(
                model.body_parentid[body_id]
            )

        return False

    with mujoco.viewer.launch_passive(model, data) as viewer:

        while viewer.is_running():

# 2D LIDAR


            if (
                data.time - last_lidar_time
                >= LIDAR_UPDATE_INTERVAL
            ):

                (
                    lidar_angles,
                    lidar_ranges
                ) = lidar.scan(
                    data
                )

                lidar_display = draw_lidar_scan(
                    lidar_angles,
                    lidar_ranges
                )

                cv2.imshow(
                    "TurtleBot 2D LiDAR",
                    lidar_display
                )

                cv2.waitKey(1)

                last_lidar_time = data.time


            
# NAVIGATION / DOCKING STATE MACHINE


# Reset reporting values
            obstacle_detected = False
            front_distance = None
            left_distance = None
            right_distance = None


            
            # MODE 1: WAYPOINT NAVIGATION
            

            if not all_waypoints_reached:

                target_waypoint = waypoints[
                    current_waypoint_index
                ]

                (
                    nominal_v,
                    nominal_omega,
                    waypoint_reached,
                    waypoint_distance
                ) = waypoint_controller.compute(
                    ekf_state,
                    target_waypoint
                )

                # Waypoint switching
                if waypoint_reached:

                    print(
                        f"\nWaypoint "
                        f"{current_waypoint_index + 1} reached."
                    )

                    current_waypoint_index += 1

                    if current_waypoint_index >= len(waypoints):

                        all_waypoints_reached = True

                        print(
                            "\nPre-docking point reached."
                        )

                        print(
                            "Switching to VISUAL DOCKING."
                        )

                if not all_waypoints_reached:

                    # Existing LiDAR-based detour navigation
                    (
                        v_command,
                        omega_command,
                        obstacle_detected,
                        front_distance,
                        left_distance,
                        right_distance
                    ) = obstacle_avoidance.compute(
                        lidar_angles,
                        lidar_ranges,
                        nominal_v,
                        nominal_omega,
                        ekf_state,
                        waypoint_controller,
                        goal_active=True,
                        primary_goal=target_waypoint
                    )

                    docking_status = "NAVIGATION"

                else:

                    # Stop during mode transition
                    v_command = 0.0
                    omega_command = 0.0

                    docking_status = "SWITCHING_TO_DOCKING"


            
            # MODE 2: VISUAL DOCKING
            

            elif not docking_complete:

                # Use only recent, valid camera measurements
                camera_valid = (
                    marker_detected
                    and base_range is not None
                    and base_bearing is not None
                    and np.isfinite(base_range)
                    and np.isfinite(base_bearing)
                    and data.time - last_camera_time <= 0.15
                )

                if camera_valid:

                    docking_range = base_range
                    docking_bearing = base_bearing

                else:

                    docking_range = None
                    docking_bearing = None

                (
                    v_command,
                    omega_command,
                    docking_complete,
                    docking_status
                ) = visual_docking.compute(
                    ekf_state,
                    docking_range,
                    docking_bearing
                )

                # LiDAR safety check during forward docking
                if (
                    not docking_complete
                    and v_command > 0.0
                    and lidar_angles is not None
                    and lidar_ranges is not None
                ):

                    front_mask = (
                        np.abs(lidar_angles)
                        <= np.deg2rad(25.0)
                    )

                    if np.any(front_mask):

                        nearest_front = np.min(
                            lidar_ranges[front_mask]
                        )

                        if nearest_front < 0.23:

                            v_command = 0.0
                            omega_command = 0.0

                            docking_status = "BLOCKED_BY_LIDAR"

                if docking_complete:

                    finish_sample_count = (
                        len(time_history) + 1
                    )

                    print(
                        "\n=========================="
                    )

                    print(
                        "DOCKING COMPLETE"
                    )

                    print(
                        "=========================="
                    )


            
            # MODE 3: COMPLETE
            

            else:

                v_command = 0.0
                omega_command = 0.0

                docking_status = "COMPLETE"


            
            # DIFFERENTIAL DRIVE
           

            omega_left, omega_right = velocity_to_wheels(
                v_command,
                omega_command
            )

            data.ctrl[left_motor_id] = omega_left
            data.ctrl[right_motor_id] = omega_right
            
            
            
           

            mujoco.mj_step(model, data)

            
# COLLISION EVALUATION ONLY


            for contact_id in range(data.ncon):

                contact = data.contact[contact_id]

                geom1 = int(contact.geom1)
                geom2 = int(contact.geom2)

                if geom1 == obstacle_geom_id:
                    other_geom_id = geom2

                elif geom2 == obstacle_geom_id:
                    other_geom_id = geom1

                else:
                    continue

                other_body_id = int(
                    model.geom_bodyid[other_geom_id]
                )

                if is_robot_body(other_body_id):

                    collision_detected = True

            



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

                
# ADD CAMERA MEASUREMENT NOISE


                if (
                    marker_detected
                    and base_range is not None
                    and base_bearing is not None
                    and np.isfinite(base_range)
                    and np.isfinite(base_bearing)
                ):

                    base_range = max(
                        0.01,
                        float(base_range) + np.random.normal(
                            0.0,
                            CAMERA_RANGE_NOISE_STD
                        )
                    )

                    noisy_bearing = (
                        float(base_bearing)
                        + np.random.normal(
                            0.0,
                            np.deg2rad(
                                CAMERA_BEARING_NOISE_STD_DEG
                            )
                        )
                    )

                    base_bearing = float(
                        np.arctan2(
                            np.sin(noisy_bearing),
                            np.cos(noisy_bearing)
                        )
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

            #ekf_state = ekf.state.copy()


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
            ekf_state = ekf.state.copy()

           
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

            v_history.append(
                float(v_command)
            )

            omega_history.append(
                float(omega_command)
            )

            camera_visible_history.append(
                int(marker_detected)
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
                        f"MODE: NAVIGATION | "
                        f"waypoint={current_waypoint_index + 1}/"
                        f"{len(waypoints)} | "
                        f"distance={waypoint_distance:.3f} m | "
                        f"v={v_command:.3f} | "
                        f"omega={omega_command:.3f}"
                    )

                elif not docking_complete:

                    print(
                        f"MODE: DOCKING | "
                        f"status={docking_status} | "
                        f"v={v_command:.3f} | "
                        f"omega={omega_command:.3f}"
                    )

                else:

                    print(
                        "MODE: COMPLETE | "
                        "Robot stopped."
                    )

                if (
                    front_distance is not None
                ):

                    print(
                        f"LIDAR: "
                        f"front={front_distance:.3f} m, "
                        f"left={left_distance:.3f} m, "
                        f"right={right_distance:.3f} m"
                    )

                    if obstacle_detected:

                        print(
                            "AVOIDANCE: obstacle detected"
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

                if all_waypoints_reached:

                    print(
                        f"ARUCO DETECTOR: "
                        f"{aruco_detector.last_detection_status}"
                    )

                if lidar_ranges is not None:

                    nearest_obstacle = np.min(
                        lidar_ranges
                    )

                    print(
                        f"LIDAR: "
                        f"nearest={nearest_obstacle:.3f} m"
                    )

                
                last_print_time = current_second
                



            viewer.sync()

            time.sleep(model.opt.timestep)


    renderer.close()
    cv2.destroyAllWindows()

    save_trial_results(
        time_history,
        true_x_history,
        true_y_history,
        true_theta_history,
        ekf_x_history,
        ekf_y_history,
        ekf_theta_history,
        sigma_x_history,
        sigma_y_history,
        sigma_theta_history,
        v_history,
        omega_history,
        camera_visible_history,
        docking_complete,
        all_waypoints_reached,
        collision_detected,
        finish_sample_count=finish_sample_count,
        scenario="final_nominal",
        seed=run_seed
    )


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
