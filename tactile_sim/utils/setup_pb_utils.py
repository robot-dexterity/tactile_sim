import argparse
import pybullet as p
import pybullet_utils.bullet_client as bc
import pkgutil
import numpy as np

from tactile_sim.assets import add_assets_path


def standard_argparse():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-e', '--embodiment_type',
        type=str,
        help="Choose task from ['arm', 'tactile_arm', 'visual_arm', 'visuotactile_arm'].",
        default='visuotactile_arm'
    )
    parser.add_argument(
        '-a', '--arm_type',
        type=str,
        help="Choose task from ['ur5', 'franka_panda', 'kuka_iiwa', 'cr3', 'mg400'].",
        default='ur5'
    )
    parser.add_argument(
        '-s', '--sensor_type',
        type=str,
        help="""Choose task from
                ['standard_tactip', 'standard_digit', 'standard_digitac', 'mini_tactip', 'flat_tactip',
                'right_angle_tactip', 'right_angle_digit', 'right_angle_digitac'].""",
        default='standard_tactip'
    )

    return parser


def connect_pybullet(timestep, show_gui=False):
    """
    Create a pyullet instance with set physics params.
    """
    if show_gui:
        pb = bc.BulletClient(connection_mode=p.GUI)
        pb.configureDebugVisualizer(pb.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        pb.configureDebugVisualizer(pb.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        pb.configureDebugVisualizer(pb.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    else:
        pb = bc.BulletClient(connection_mode=p.DIRECT)
        egl = pkgutil.get_loader("eglRenderer")
        if egl:
            p.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
        else:
            p.loadPlugin("eglRendererPlugin")

    pb.setGravity(0, 0, -9.81)
    pb.setPhysicsEngineParameter(
        fixedTimeStep=timestep,
        numSolverIterations=300,
        numSubSteps=1,
        contactBreakingThreshold=0.0005,
        erp=0.05,
        contactERP=0.05,
        frictionERP=0.2,
        solverResidualThreshold=1e-7,
        contactSlop=0.001,
        globalCFM=0.0001,
    )
    return pb


def load_standard_environment(pb):
    """
    Load a standard environment with a plane and a table.
    """
    pb.loadURDF(
        add_assets_path("shared_assets/environment_objects/plane/plane.urdf"),
        [0, 0, -0.625],
    )
    pb.loadURDF(
        add_assets_path("shared_assets/environment_objects/table/table.urdf"),
        [0.50, 0.00, -0.625],
        [0.0, 0.0, 0.0, 1.0],
    )


def load_stim(pb, stim_name, stim_pose, fixed_base=False, enable_collision=True, scale=1.0):
    """
    Load a stimulus at a given pose.
    """
    stim_pose = np.array(stim_pose)
    stim_pos, stim_rpy = stim_pose[:3], stim_pose[3:]

    stim_id = p.loadURDF(
        stim_name,
        stim_pos,
        pb.getQuaternionFromEuler(stim_rpy),
        useFixedBase=fixed_base,
        globalScaling=scale
    )

    if not enable_collision:
        pb.setCollisionFilterGroupMask(stim_id, -1, 0, 0)

    if not fixed_base:
        p.changeDynamics(stim_id, -1, mass=0.05)

    return stim_id


def load_target_indicator(pb, target_pose):
    target_pose = np.array(target_pose)
    target_pos, target_rpy = target_pose[:3], target_pose[3:]
    target_id = p.loadURDF(
        add_assets_path("shared_assets/environment_objects/goal_indicators/sphere_indicator.urdf"),
        target_pos,
        pb.getQuaternionFromEuler(target_rpy),
        useFixedBase=True,
    )
    p.changeVisualShape(target_id, -1, rgbaColor=[0, 1, 0, 0.5])
    p.setCollisionFilterGroupMask(target_id, -1, 0, 0)
    return target_id


def set_debug_camera(pb, debug_camera_params):
    pb.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    pb.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    pb.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    pb.resetDebugVisualizerCamera(
        debug_camera_params['dist'],
        debug_camera_params['yaw'],
        debug_camera_params['pitch'],
        debug_camera_params['pos']
    )


def add_tcp_user_control(pb):
    """create controllable parameters on GUI"""

    min_pos_action, max_pos_action = -0.01, 0.01
    min_rot_action, max_rot_action = np.deg2rad(-5.0), np.deg2rad(5.0)

    action_ids = []
    action_ids.append(pb.addUserDebugParameter("dx", min_pos_action, max_pos_action, 0))
    action_ids.append(pb.addUserDebugParameter("dy", min_pos_action, max_pos_action, 0))
    action_ids.append(pb.addUserDebugParameter("dz", min_pos_action, max_pos_action, 0))
    action_ids.append(pb.addUserDebugParameter("dRX", min_rot_action, max_rot_action, 0))
    action_ids.append(pb.addUserDebugParameter("dRY", min_rot_action, max_rot_action, 0))
    action_ids.append(pb.addUserDebugParameter("dRZ", min_rot_action, max_rot_action, 0))
    return action_ids


def add_joint_user_control(pb, control_joints):
    """create controllable parameters on GUI"""

    min_action, max_action = np.deg2rad(-1.0), np.deg2rad(1.0)

    action_ids = []
    for joint_name in control_joints:
        action_ids.append(pb.addUserDebugParameter(joint_name, min_action, max_action, 0))

    return action_ids


def simple_pb_loop():
    p.setRealTimeSimulation(1)
    while p.isConnected():
        q_key = ord("q")
        keys = p.getKeyboardEvents()
        if q_key in keys and keys[q_key] & p.KEY_WAS_TRIGGERED:
            exit()
