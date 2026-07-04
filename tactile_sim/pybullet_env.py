import os

import numpy as np

from tactile_sim.assets import add_assets_path
from tactile_sim.assets.default_rest_poses import rest_poses_dict
from tactile_sim.embodiments import create_embodiment
from tactile_sim.utils.setup_pb_utils import (
    connect_pybullet,
    load_standard_environment,
    load_stim,
    load_target_indicator,
    set_debug_camera,
    simple_pb_loop,
)


def pybullet_env(
    embodiment_type="tactile_arm",
    arm_type="ur",
    sensor_type="standard_tactip",
    sensor_core="no_core",
    sensor_dynamics=None,
    image_size=(128, 128),
    show_tactile=False,
    stim_name="circle",
    stim_path=None,
    stim_pose=(600, 0, 12.5, 0, 0, 0),
    show_gui=True,
    load_target=None,
    **kwargs
):
    timestep = 1 / 240.0
    sensor_dynamics = sensor_dynamics or {}
    stim_path = stim_path or add_assets_path("stimuli")

    arm_mapping = {
        "ur": "ur5",
        "franka": "franka_panda",
        "kuka": "kuka_iiwa",
        "mg400": "mg400",
        "cr": "cr3",
    }
    arm_type = arm_mapping.get(arm_type, arm_type)

    robot_arm_params = {
        "type": arm_type,
        "rest_poses": rest_poses_dict[arm_type],
        "tcp_lims": np.column_stack([-np.inf * np.ones(6), np.inf * np.ones(6)]),
    }

    tactile_sensor_params = {
        "type": sensor_type,
        "core": sensor_core,
        "dynamics": sensor_dynamics,
        "image_size": image_size,
        "turn_off_border": False,
        "show_tactile": show_tactile,
    }

    visual_sensor_params = {
        "image_size": [128, 128],
        "dist": 0.25,
        "yaw": 90.0,
        "pitch": -25.0,
        "pos": [0.6, 0.0, 0.0525],
        "fov": 75.0,
        "near_val": 0.1,
        "far_val": 100.0,
        "show_vision": False,
    }

    pb = connect_pybullet(timestep, show_gui)
    load_standard_environment(pb)

    if stim_name is not None:
        stim_urdf = os.path.join(stim_path, stim_name, f"{stim_name}.urdf")
        load_stim(pb, stim_urdf, np.array(stim_pose) / 1e3, fixed_base=True, enable_collision=True)
    if load_target is not None:
        load_target_indicator(pb, load_target)

    embodiment = create_embodiment(
        pb,
        embodiment_type,
        robot_arm_params,
        tactile_sensor_params,
        visual_sensor_params,
    )
    set_debug_camera(pb, visual_sensor_params)
    return embodiment


if __name__ == "__main__":
    from cri.robot import SyncRobot
    from cri.sim.sim_controller import SimController

    parameters = {
        "arm_type": "ur",
        "stim_name": "circle",
        "work_frame": (600, 0, 200, 0, 0, 0),
        "tcp_pose": (600, 0, 0, 0, 0, 0),
        "stim_pose": (600, 0, 0, 0, 0, 0),
        "show_gui": True,
        "type": "standard_tactip",
        "image_size": (256, 256),
        "show_tactile": False,
    }

    embodiment = pybullet_env(**parameters)
    robot = SyncRobot(SimController(embodiment.arm))
    simple_pb_loop()
