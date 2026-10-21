
from src.control.differential_drive import velocity_to_wheels
import time
from pathlib import Path

import mujoco
import mujoco.viewer


MODEL_PATH = (
    Path(__file__).parent
    / "models"
    / "turtlebot3"
    / "scene_turtlebot3_waffle_pi.xml"
)


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

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

    with mujoco.viewer.launch_passive(model, data) as viewer:

        while viewer.is_running():

            
            
            v_command = 0.0
            omega_command = 0.5

            omega_left, omega_right = velocity_to_wheels(
                v_command,
                omega_command
            )

            data.ctrl[left_motor_id] = omega_left
            data.ctrl[right_motor_id] = omega_right
            
            
            
           

            mujoco.mj_step(model, data)

            viewer.sync()

            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()