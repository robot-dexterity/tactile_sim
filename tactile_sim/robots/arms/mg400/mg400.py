import numpy as np

from tactile_sim.robots.arms.base_robot_arm import BaseRobotArm


class MG400(BaseRobotArm):
    def __init__(
        self,
        pb,
        embodiment_id,
        tcp_link_id,
        link_name_to_index,
        joint_name_to_index,
        rest_poses
    ):
        super(MG400, self).__init__(
            pb, embodiment_id, tcp_link_id, link_name_to_index, joint_name_to_index, rest_poses
        )

        # set info specific to arm
        self.setup_mg400_info()
        self.robot_type = 'MG400'

        # reset the arm to rest poses
        self.reset()

    def setup_mg400_info(self):
        """
        Set some of the parameters used when controlling the mg400
        """
        self.name = 'sim_mg400'
        self.max_force = 1000.0
        self.pos_gain = 1.0
        self.vel_gain = 1.0

        self.num_joints = self._pb.getNumJoints(self.embodiment_id)

        # joints which can be controlled (not fixed)
        self.control_joint_names = [
            "j1",
            "j2_1",
            "j3_1",
            "j4_1",
            "j5",
            "j2_2",
            "j3_2",
            "j4_2",
        ]

        # get the control and calculate joint ids in list form, useful for pb array methods
        self.control_joint_ids = [self.joint_name_to_index[name] for name in self.control_joint_names]
        self.num_control_dofs = len(self.control_joint_ids)

    def _logical_to_control_joint_angles(self, joint_angles):
        """
        Expand the MG400's 4 logical joints to the coupled PyBullet joints.
        """
        joint_angles = np.array(joint_angles, dtype=float).ravel()

        if len(joint_angles) == self.num_control_dofs:
            return joint_angles
        if len(joint_angles) != 4:
            raise ValueError(f"MG400 expects 4 or {self.num_control_dofs} joint angles")

        control_joint_angles = super().get_joint_angles()
        control_joint_angles[:4] = joint_angles
        control_joint_angles[5] = joint_angles[1]
        control_joint_angles[6] = -joint_angles[1]
        control_joint_angles[7] = joint_angles[1] + joint_angles[2]
        return control_joint_angles

    def _process_ik_joint_positions(self, joint_positions):
        """Unwrap J1 and enforce parallel-link coupling on an IK solution."""
        joint_positions = np.asarray(joint_positions, dtype=float).copy()
        current_j1 = super().get_joint_angles()[0]
        joint_positions[0] += 2 * np.pi * np.round(
            (current_j1 - joint_positions[0]) / (2 * np.pi)
        )
        joint_positions[5] = joint_positions[1]
        joint_positions[6] = -joint_positions[1]
        joint_positions[7] = joint_positions[1] + joint_positions[2]

        for name, joint_id, target in zip(
            self.control_joint_names, self.control_joint_ids, joint_positions
        ):
            joint_info = self._pb.getJointInfo(self.embodiment_id, joint_id)
            lower_limit, upper_limit = joint_info[8], joint_info[9]
            if lower_limit <= upper_limit and not lower_limit <= target <= upper_limit:
                raise ValueError(
                    f"MG400 IK solution violates {name} limits: "
                    f"target={target:.5f}, "
                    f"limits=[{lower_limit:.5f}, {upper_limit:.5f}]"
                )

        return joint_positions

    def get_joint_angles(self):
        """
        Return the MG400's 4 logical joints, matching the real robot interface.
        """
        return super().get_joint_angles()[:4]

    def move_joints(self, targ_joint_angles, quick_mode=False):
        targ_joint_angles = self._logical_to_control_joint_angles(targ_joint_angles)
        super().move_joints(targ_joint_angles, quick_mode=quick_mode)

    def tcp_velocity_control(self, desired_vels):
        """
        Actions specifiy desired velocities in the workframe.
        TCP limits are imposed.

        Jacobian size is irregular so alwys use psuedo inverse
        """
        # check that this won't push the TCP out of limits
        # zero any velocities that will
        capped_desired_vels = self.check_TCP_vel_lims(np.array(desired_vels))

        # convert desired vels from workframe to worldframe
        capped_desired_vels[:3], capped_desired_vels[3:] = self.workvel_to_worldvel(
            capped_desired_vels[:3], capped_desired_vels[3:]
        )

        # get current joint positions and velocities
        q, qd = self.get_current_joint_pos_vel()

        # calculate the jacobian for tcp link
        # used to map joing velocities to TCP velocities
        jac_t, jac_r = self._pb.calculateJacobian(
            self.embodiment_id,
            self.tcp_link_id,
            [0, 0, 0],
            q,
            qd,
            [0] * self.num_control_dofs,
        )

        # merge into one jacobian matrix
        jac = np.concatenate([np.array(jac_t), np.array(jac_r)])

        # invert the jacobian to map from tcp velocities to joint velocities
        inv_jac = np.linalg.pinv(jac)

        # convert desired velocities from cart space to joint space
        req_joint_vels = np.matmul(inv_jac, capped_desired_vels)
        if self.robot_type == "MG400":
            joint_poses = list(req_joint_vels)
            joint_poses[-3] = joint_poses[1]
            joint_poses[-2] = -joint_poses[1]
            joint_poses[-1] = joint_poses[1] + joint_poses[2]
            req_joint_vels = tuple(joint_poses)

        # apply joint space velocities
        self._pb.setJointMotorControlArray(
            self.embodiment_id,
            self.control_joint_ids,
            self._pb.VELOCITY_CONTROL,
            targetVelocities=req_joint_vels,
            velocityGains=[self.vel_gain] * self.num_control_dofs,
            forces=[self.max_force] * self.num_control_dofs,
        )

    def tcp_position_control(self, desired_delta_pose):
        """
        Actions specifiy desired changes in position in the workframe.
        TCP limits are imposed.
        """
        # get current position
        (
            cur_tcp_pos,
            cur_tcp_rpy,
            cur_tcp_orn,
            _,
            _,
        ) = self.get_current_TCP_pos_vel_workframe()

        # add actions to current positions
        target_pos = cur_tcp_pos + np.array(desired_delta_pose[:3])
        target_rpy = cur_tcp_rpy + np.array(desired_delta_pose[3:])

        # limit actions to safe ranges
        target_pos, target_rpy = self.check_TCP_pos_lims(target_pos, target_rpy)

        # convert to worldframe coords for IK
        target_pos, target_rpy = self.workframe_to_worldframe(target_pos, target_rpy)
        target_orn = self._pb.getQuaternionFromEuler(target_rpy)

        # get joint positions using inverse kinematics
        joint_poses = self._pb.calculateInverseKinematics(
            self.embodiment_id,
            self.tcp_link_id,
            target_pos,
            target_orn,
            restPoses=self.rest_poses,
            maxNumIterations=100,
            residualThreshold=1e-8,
        )

        if self.robot_type == "MG400":
            joint_poses = list(joint_poses)
            joint_poses[-3] = joint_poses[1]
            joint_poses[-2] = -joint_poses[1]
            joint_poses[-1] = joint_poses[1] + joint_poses[2]
            joint_poses = tuple(joint_poses)

        # set joint control
        self._pb.setJointMotorControlArray(
            self.embodiment_id,
            self.control_joint_ids,
            self._pb.POSITION_CONTROL,
            targetPositions=joint_poses,
            targetVelocities=[0] * self.num_control_dofs,
            positionGains=[self.pos_gain] * self.num_control_dofs,
            velocityGains=[self.vel_gain] * self.num_control_dofs,
            forces=[self.max_force] * self.num_control_dofs,
        )

        # set target positions for blocking move
        self.target_pos_worldframe = target_pos
        self.target_rpy_worldframe = target_rpy
        self.target_orn_worldframe = target_orn
        self.target_joints = joint_poses

    def tcp_direct_workframe_move(self, target_pos, target_rpy):
        """
        Go directly to a position specified relative to the workframe
        """

        # transform from work_frame to world_frame
        target_pos, target_rpy = self.workframe_to_worldframe(target_pos, target_rpy)
        target_orn = np.array(self._pb.getQuaternionFromEuler(target_rpy))

        # get target joint poses through IK
        joint_poses = self._pb.calculateInverseKinematics(
            self.embodiment_id,
            self.tcp_link_id,
            target_pos,
            target_orn,
            restPoses=self.rest_poses,
            maxNumIterations=100,
            residualThreshold=1e-8,
        )
        # set joint control
        self._pb.setJointMotorControlArray(
            self.embodiment_id,
            self.control_joint_ids,
            self._pb.POSITION_CONTROL,
            targetPositions=joint_poses,
            targetVelocities=[0] * self.num_control_dofs,
            positionGains=[self.pos_gain] * self.num_control_dofs,
            velocityGains=[self.vel_gain] * self.num_control_dofs,
            forces=[self.max_force] * self.num_control_dofs,
        )
        # set target positions for blocking move
        if self.robot_type == "MG400":
            joint_poses = list(joint_poses)
            joint_poses[-3] = joint_poses[1]
            joint_poses[-2] = -joint_poses[1]
            joint_poses[-1] = joint_poses[1] + joint_poses[2]
            joint_poses = tuple(joint_poses)

        self.target_pos_worldframe = target_pos
        self.target_rpy_worldframe = target_rpy
        self.target_orn_worldframe = target_orn
        self.target_joints = joint_poses
