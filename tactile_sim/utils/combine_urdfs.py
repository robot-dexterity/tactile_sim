import os
import time
import numpy as np
from glob import glob
import pathlib
from tempfile import mkstemp
import shutil

import pybullet as pb
from pybullet_utils import bullet_client as bc
from pybullet_utils import urdfEditor as ed

from tactile_sim.assets import add_assets_path


def combine_urdfs(arm_type, sensor_type, joint_pivot_xyz, joint_pivot_rpy, save_urdf=True, show_combined_urdf=False):
    """
    Combine sensor and arm urdfs.
    """

    base_pos = [0, 0, 0]
    base_rpy = [0, 0, 0]
    base_orn = pb.getQuaternionFromEuler(base_rpy)

    arm_client = bc.BulletClient(connection_mode=pb.DIRECT)
    sensor_client = bc.BulletClient(connection_mode=pb.DIRECT)

    arm_asset_name = os.path.join(
            "robot_arm_assets",
            arm_type,
            "urdfs",
            arm_type + ".urdf",
        )
    sensor_asset_name = os.path.join(
            "sensor_assets",
            sensor_type,
            sensor_type + ".urdf",
        )
    arm = arm_client.loadURDF(add_assets_path(arm_asset_name))
    sensor = sensor_client.loadURDF(add_assets_path(sensor_asset_name))

    arm_editor = ed.UrdfEditor()
    arm_editor.initializeFromBulletBody(arm, arm_client._client)
    sensor_editor = ed.UrdfEditor()
    sensor_editor.initializeFromBulletBody(sensor, sensor_client._client)

    # find ee index
    for i in range(pb.getNumJoints(arm)):
        info = pb.getJointInfo(arm, i)
        link_name = info[12].decode("utf-8")
        if link_name == "ee_link":
            ee_index = i+1

    newjoint = arm_editor.joinUrdf(
            sensor_editor,
            ee_index,
            joint_pivot_xyz,
            joint_pivot_rpy,
            [0, 0, 0],
            [0, 0, 0],
            arm_client._client,
            sensor_client._client
        )
    newjoint.joint_type = arm_client.JOINT_FIXED

    if save_urdf:
        combined_asset_path = add_assets_path(os.path.join(
            "embodiment_assets",
            "combined_urdfs",
        ))
        os.makedirs(combined_asset_path, exist_ok=True)
        combined_asset_name = os.path.join(
            combined_asset_path,
            arm_type + "_" + sensor_type + ".urdf",
        )
        arm_editor.saveUrdf(combined_asset_name)

        # fix bug when saving materials
        adjust_materials(combined_asset_name)

    if show_combined_urdf:
        pgui = bc.BulletClient(connection_mode=pb.GUI)
        arm_editor.createMultiBody(base_pos, base_orn, pgui._client)
        pgui.setRealTimeSimulation(1)

        while (pgui.isConnected()):
            time.sleep(1. / 240.)


def adjust_materials(urdf_path):
    """
    Needed to fix bug where combined urdfs contain overlapping materials.
    Apply to single urdf.
    """

    # create temp file to copy into
    temp_fh, temp_path = mkstemp()

    path_str = add_assets_path("")
    mat_str = "<material name=\"mat_"
    mat_counter = 0

    with os.fdopen(temp_fh, 'w') as new_file:
        with open(urdf_path, 'r') as old_file:
            for line in old_file.readlines():

                # change xml version for editor formatting
                if line == "<?xml version=\"0.0\" ?>\n":
                    line = "<?xml version=\"1.0\" ?>\n"

                # change absolute path to relative
                if path_str in line:
                    line = line.replace(path_str, "..")

                # change the materials to be unique in combined urdfs
                if mat_str in line:
                    line = f"\t\t\t<material name=\"mat_{mat_counter}\">\n"
                    mat_counter += 1

                new_file.write(line)

    # copy the file permissions from the old file to the new file
    shutil.copymode(urdf_path, temp_path)

    # remove original file
    os.remove(urdf_path)

    # move new file
    shutil.move(temp_path, urdf_path)


def adjust_all_materials(base_dir):
    """
    Needed to fix bug where combined urdfs contain overlapping materials.
    apply to directory of urdfs
    """
    path = pathlib.Path(base_dir).absolute()
    all_urdf_files = [y for x in os.walk(path) for y in glob(os.path.join(x[0], '*.urdf'))]
    for file_path in all_urdf_files:
        path = pathlib.Path(file_path)
        print('Adjusting: ', path)
        adjust_materials(path)


if __name__ == '__main__':

    # adjust_all_materials(add_assets_path(os.path.join("embodiment_assets", "combined_urdfs")))

    save_urdf = True

    # standard tactip
    # combine_urdfs('ur5', 'standard_tactip', [0.0, 0.0, 0.0], [-np.pi, 0.0, np.pi], save_urdf=save_urdf)
    # combine_urdfs('franka_panda', 'standard_tactip', [0.0, 0.0, 0.0], [-np.pi, 0.0, np.pi], save_urdf=save_urdf)
    # combine_urdfs('kuka_iiwa', 'standard_tactip', [0.0, 0.0, 0.0], [-np.pi, 0.0, np.pi], save_urdf=save_urdf)
    # combine_urdfs('cr3', 'standard_tactip', [0.0, 0.0, 0.0], [-np.pi, 0.0, np.pi], save_urdf=save_urdf)
    # combine_urdfs('mg400', 'standard_tactip', [0.0, 0.0, 0.0], [-np.pi, 0.0, np.pi], save_urdf=save_urdf)

    # standard digit
    # combine_urdfs('ur5', 'standard_digit', [0.0, 0.0, 0.0], [-np.pi, 0.0, np.pi], save_urdf=save_urdf)
    # combine_urdfs('franka_panda', 'standard_digit', [0.0, 0.0, 0.0], [-np.pi, 0.0, np.pi], save_urdf=save_urdf)
    # combine_urdfs('kuka_iiwa', 'standard_digit', [0.0, 0.0, 0.0], [-np.pi, 0.0, np.pi], save_urdf=save_urdf)
    # combine_urdfs('cr3', 'standard_digit', [0.0, 0.0, 0.0], [-np.pi, 0.0, np.pi], save_urdf=save_urdf)
    # combine_urdfs('mg400', 'standard_digit', [0.0, 0.0, 0.0], [-np.pi, 0.0, np.pi], save_urdf=save_urdf)

    # standard digitac
    # combine_urdfs('ur5', 'standard_digitac', [0.0, 0.0, 0.0], [-np.pi, 0.0, np.pi], save_urdf=save_urdf)
    # combine_urdfs('franka_panda', 'standard_digitac', [0.0, 0.0, 0.0], [-np.pi, 0.0, np.pi], save_urdf=save_urdf)
    # combine_urdfs('kuka_iiwa', 'standard_digitac', [0.0, 0.0, 0.0], [-np.pi, 0.0, np.pi], save_urdf=save_urdf)
    # combine_urdfs('cr3', 'standard_digitac', [0.0, 0.0, 0.0], [-np.pi, 0.0, np.pi], save_urdf=save_urdf)
    # combine_urdfs('mg400', 'standard_digitac', [0.0, 0.0, 0.0], [-np.pi, 0.0, np.pi], save_urdf=save_urdf)

    # mini tactip
    # combine_urdfs('ur5', 'mini_tactip', [0.0, 0.0, 0.0], [-np.pi, 0.0, np.pi], save_urdf=save_urdf)
    # combine_urdfs('franka_panda', 'mini_tactip', [0.0, 0.0, 0.0], [-np.pi, 0.0, np.pi], save_urdf=save_urdf)
    # combine_urdfs('kuka_iiwa', 'mini_tactip', [0.0, 0.0, 0.0], [-np.pi, 0.0, np.pi], save_urdf=save_urdf)
    # combine_urdfs('cr3', 'mini_tactip', [0.0, 0.0, 0.0], [-np.pi, 0.0, np.pi], save_urdf=save_urdf)
    # combine_urdfs('mg400', 'mini_tactip', [0.0, 0.0, 0.0], [-np.pi, 0.0, np.pi], save_urdf=save_urdf)

    # flat tactip
    combine_urdfs('ur5', 'flat_tactip', [0.0, 0.0, 0.0], [-np.pi, 0.0, np.pi], save_urdf=save_urdf)
    combine_urdfs('franka_panda', 'flat_tactip', [0.0, 0.0, 0.0], [-np.pi, 0.0, np.pi], save_urdf=save_urdf)
    combine_urdfs('kuka_iiwa', 'flat_tactip', [0.0, 0.0, 0.0], [-np.pi, 0.0, np.pi], save_urdf=save_urdf)
    combine_urdfs('cr3', 'flat_tactip', [0.0, 0.0, 0.0], [-np.pi, 0.0, np.pi], save_urdf=save_urdf)
    combine_urdfs('mg400', 'flat_tactip', [0.0, 0.0, 0.0], [-np.pi, 0.0, np.pi], save_urdf=save_urdf)

    # right angle tactips
    # combine_urdfs('ur5', 'right_angle_tactip', [0.0, 0.005, -0.06], [-np.pi/2, -np.pi/2, np.pi], save_urdf=save_urdf)
    # combine_urdfs('franka_panda', 'right_angle_tactip', [0, 0.005, -0.06], [-np.pi/2, -np.pi/2, np.pi], save_urdf=save_urdf)
    # combine_urdfs('kuka_iiwa', 'right_angle_tactip', [0.0, 0.005, -0.06], [-np.pi/2, -np.pi/2, np.pi], save_urdf=save_urdf)
    # combine_urdfs('cr3', 'right_angle_tactip', [0.0, 0.005, -0.06], [-np.pi/2, -np.pi/2, np.pi], save_urdf=save_urdf)
    # combine_urdfs('mg400', 'right_angle_tactip', [0.0, 0.005, -0.06], [-np.pi/2, -np.pi/2, np.pi/2], save_urdf=save_urdf)

    # right angle digits
    # combine_urdfs('ur5', 'right_angle_digit', [0.0, 0.005, -0.06], [-np.pi/2, -np.pi/2, np.pi], save_urdf=save_urdf)
    # combine_urdfs('franka_panda', 'right_angle_digit', [0.0, 0.005, -0.06], [-np.pi/2, -np.pi/2, np.pi], save_urdf=save_urdf)
    # combine_urdfs('kuka_iiwa', 'right_angle_digit', [0.0, 0.005, -0.06], [-np.pi/2, -np.pi/2, np.pi], save_urdf=save_urdf)
    # combine_urdfs('cr3', 'right_angle_digit', [0.0, 0.005, -0.06], [-np.pi/2, -np.pi/2, np.pi], save_urdf=save_urdf)
    # combine_urdfs('mg400', 'right_angle_digit', [0.0, 0.005, -0.06], [-np.pi/2, -np.pi/2, np.pi/2], save_urdf=save_urdf)

    # right angle digitacs
    # combine_urdfs('ur5', 'right_angle_digitac', [0.0, 0.005, -0.06], [-np.pi/2, -np.pi/2, np.pi], save_urdf=save_urdf)
    # combine_urdfs('franka_panda', 'right_angle_digitac', [0.0, 0.005, -0.06], [-np.pi/2, -np.pi/2, np.pi], save_urdf=save_urdf)
    # combine_urdfs('kuka_iiwa', 'right_angle_digitac', [0.0, 0.005, -0.06], [-np.pi/2, -np.pi/2, np.pi], save_urdf=save_urdf)
    # combine_urdfs('cr3', 'right_angle_digitac', [0.0, 0.005, -0.06], [-np.pi/2, -np.pi/2, np.pi], save_urdf=save_urdf)
    # combine_urdfs('mg400', 'right_angle_digitac', [0.0, 0.005, -0.06], [-np.pi/2, -np.pi/2, np.pi/2], save_urdf=save_urdf)
