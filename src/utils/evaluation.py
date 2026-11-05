import json
from datetime import datetime
from pathlib import Path

import numpy as np

from config import (
    ARUCO_WORLD_X,
    ARUCO_WORLD_Y,
    DOCK_TARGET_BASE_RANGE,
    DOCK_RANGE_TOLERANCE,
    DOCK_BEARING_TOLERANCE_DEG
)


def wrap_angle(angle):
    return np.arctan2(
        np.sin(angle),
        np.cos(angle)
    )


def save_trial_results(
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
    finish_sample_count=None,
    scenario="nominal",
    seed=2026
):

    histories = {
        "time_s": time_history,
        "true_x": true_x_history,
        "true_y": true_y_history,
        "true_theta": true_theta_history,
        "ekf_x": ekf_x_history,
        "ekf_y": ekf_y_history,
        "ekf_theta": ekf_theta_history,
        "sigma_x": sigma_x_history,
        "sigma_y": sigma_y_history,
        "sigma_theta": sigma_theta_history,
        "command_v": v_history,
        "command_omega": omega_history,
        "camera_visible": camera_visible_history
    }

    if finish_sample_count is None:
        n = len(time_history)
    else:
        n = int(finish_sample_count)

    if n < 2:
        raise ValueError("Not enough evaluation samples.")

    for name, values in histories.items():
        if len(values) < n:
            raise ValueError(
                f"Incomplete history: {name}"
            )

    arrays = {
        name: np.asarray(values[:n], dtype=float)
        for name, values in histories.items()
    }

    for name, values in arrays.items():
        if not np.all(np.isfinite(values)):
            raise ValueError(
                f"Invalid values in {name}"
            )

    t = arrays["time_s"]

    tx = arrays["true_x"]
    ty = arrays["true_y"]
    tt = arrays["true_theta"]

    ex = arrays["ekf_x"]
    ey = arrays["ekf_y"]
    et = arrays["ekf_theta"]

    sx = arrays["sigma_x"]
    sy = arrays["sigma_y"]
    st = arrays["sigma_theta"]

    v = arrays["command_v"]
    omega = arrays["command_omega"]

    
    # ESTIMATION ERRORS
    

    x_error = tx - ex
    y_error = ty - ey

    position_error = np.hypot(
        x_error,
        y_error
    )

    heading_error = wrap_angle(
        tt - et
    )

    position_rmse = np.sqrt(
        np.mean(position_error ** 2)
    )

    heading_rmse = np.sqrt(
        np.mean(heading_error ** 2)
    )

    
    # UNCERTAINTY COVERAGE
    

    x_coverage = 100.0 * np.mean(
        np.abs(x_error) <= 2.0 * sx
    )

    y_coverage = 100.0 * np.mean(
        np.abs(y_error) <= 2.0 * sy
    )

    heading_coverage = 100.0 * np.mean(
        np.abs(heading_error) <= 2.0 * st
    )

    
    # PATH LENGTH AND CONTROL EFFORT
    

    path_length = np.sum(
        np.hypot(
            np.diff(tx),
            np.diff(ty)
        )
    )

    mean_abs_v = np.mean(
        np.abs(v)
    )

    mean_abs_omega = np.mean(
        np.abs(omega)
    )

    
    # FINAL DOCKING ACCURACY
    # Ground Truth used for evaluation only
    

    dx = ARUCO_WORLD_X - tx[-1]
    dy = ARUCO_WORLD_Y - ty[-1]

    final_true_range = np.hypot(
        dx,
        dy
    )

    final_true_bearing = wrap_angle(
        np.arctan2(dy, dx) - tt[-1]
    )

    docking_range_error = abs(
        final_true_range
        - DOCK_TARGET_BASE_RANGE
    )

    docking_bearing_error_deg = abs(
        np.degrees(final_true_bearing)
    )

    range_ok = (
        docking_range_error
        <= DOCK_RANGE_TOLERANCE
    )

    bearing_ok = (
        docking_bearing_error_deg
        <= DOCK_BEARING_TOLERANCE_DEG
    )

    ground_truth_docking_ok = bool(
        range_ok and bearing_ok
    )

    # Strict geometric success definition
    trial_success = bool(
        docking_complete
        and all_waypoints_reached
        and ground_truth_docking_ok
        and not collision_detected
    )

    
    # SUMMARY
    

    summary = {
        "scenario": scenario,
        "seed": int(seed),

        "software_docking_complete": bool(
            docking_complete
        ),

        "all_waypoints_reached": bool(
            all_waypoints_reached
        ),

        "collision_detected": bool(
            collision_detected
        ),

        "trial_success": trial_success,

        "duration_s": float(
            t[-1] - t[0]
        ),

        "position_rmse_m": float(
            position_rmse
        ),

        "heading_rmse_rad": float(
            heading_rmse
        ),

        "x_2sigma_coverage_percent": float(
            x_coverage
        ),

        "y_2sigma_coverage_percent": float(
            y_coverage
        ),

        "heading_2sigma_coverage_percent": float(
            heading_coverage
        ),

        "actual_path_length_m": float(
            path_length
        ),

        "mean_abs_command_v_mps": float(
            mean_abs_v
        ),

        "mean_abs_command_omega_radps": float(
            mean_abs_omega
        ),

        "camera_visible_percent": float(
            100.0 * np.mean(
                arrays["camera_visible"]
            )
        ),

        "final_true_range_m": float(
            final_true_range
        ),

        "final_true_bearing_deg": float(
            np.degrees(final_true_bearing)
        ),

        "docking_range_error_m": float(
            docking_range_error
        ),

        "docking_bearing_error_deg": float(
            docking_bearing_error_deg
        ),

        "ground_truth_docking_ok": (
            ground_truth_docking_ok
        )
    }

    
    # SAVE FILES
    

    output_dir = (
        Path(__file__).resolve().parents[2]
        / "results"
    )

    output_dir.mkdir(
        parents=True,
        exist_ok=True
    )

    run_id = datetime.now().strftime(
        "%Y%m%d_%H%M%S_%f"
    )

    filename = (
        f"{scenario}_seed{seed}_{run_id}"
    )

    csv_path = output_dir / (
        filename + ".csv"
    )

    json_path = output_dir / (
        filename + ".json"
    )

    column_names = list(arrays.keys())

    np.savetxt(
        csv_path,
        np.column_stack(
            [arrays[name] for name in column_names]
        ),
        delimiter=",",
        header=",".join(column_names),
        comments=""
    )

    with open(
        json_path,
        "w",
        encoding="utf-8"
    ) as f:

        json.dump(
            summary,
            f,
            indent=4
        )

    print("\n===== TRIAL EVALUATION =====")

    for key, value in summary.items():
        print(f"{key}: {value}")

    print("============================\n")

    print(f"CSV saved: {csv_path}")
    print(f"JSON saved: {json_path}")

    return summary