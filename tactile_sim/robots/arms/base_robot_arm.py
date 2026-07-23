import numpy as np
import warnings
from tactile_sim.utils.pybullet_draw_utils import draw_link_frame

warnings.simplefilter('always', UserWarning)


class BaseRobotArm:
    def __init__(
        self,
        pb,
        embodiment_id,
        tcp_link_id,
        link_name_to_index,
        joint_name_to_index,
        rest_poses,
    ):

        self._pb = pb
        self.rest_poses = rest_poses  # default joint pose for ur5
        self.embodiment_id = embodiment_id
        self.num_joints = self._pb.getNumJoints(embodiment_id)
        self.tcp_link_id = tcp_link_id
        self.link_name_to_index = link_name_to_index
        self.joint_name_to_index = joint_name_to_index

        self._min_constant_vel = 0.0001
        self._max_constant_vel = 0.001
        self.set_constant_vel_percentage(percentage=25)

    def close(self):
        if self._pb.isConnected():
            self._pb.disconnect()

    def reset(self):
        """
        Reset the UR5 to its rest positions and hold.
        """
        # reset the joint positions to a rest position
        for i in range(self.num_joints):
            self._pb.resetJointState(self.embodiment_id, i, self.rest_poses[i])
            self._pb.changeDynamics(self.embodiment_id, i, linearDamping=0.04, angularDamping=0.04)
            self._pb.changeDynamics(self.embodiment_id, i, jointDamping=0.01)

        # hold in rest pose
        self._pb.setJointMotorControlArray(
            bodyIndex=self.embodiment_id,
            jointIndices=self.control_joint_ids,
            controlMode=self._pb.POSITION_CONTROL,
            targetPositions=self.rest_poses[self.control_joint_ids],
            targetVelocities=[0] * self.num_control_dofs,
            positionGains=[self.pos_gain] * self.num_control_dofs,
            velocityGains=[self.vel_gain] * self.num_control_dofs,
            forces=np.zeros(self.num_control_dofs) + self.max_force,
        )

    def get_current_joint_pos_vel(self):
        """
        Get the current joint states of the ur5
        """
        cur_joint_states = self._pb.getJointStates(self.embodiment_id, self.control_joint_ids)
        cur_joint_pos = [cur_joint_states[i][0] for i in range(self.num_control_dofs)]
        cur_joint_vel = [cur_joint_states[i][1] for i in range(self.num_control_dofs)]
        return cur_joint_pos, cur_joint_vel

    def get_current_tcp_pose_vel(self):
        """
        Get the current velocity of the TCP
        """
        tcp_state = self._pb.getLinkState(
            self.embodiment_id,
            self.tcp_link_id,
            computeLinkVelocity=True,
            computeForwardKinematics=False,
        )
        tcp_pos = np.array(tcp_state[0])  # worldLinkPos
        tcp_orn = np.array(tcp_state[1])  # worldLinkOrn
        tcp_rpy = self._pb.getEulerFromQuaternion(tcp_orn)
        tcp_pose = np.array([*tcp_pos, *tcp_rpy])

        tcp_lin_vel = np.array(tcp_state[6])  # worldLinkLinearVelocity
        tcp_ang_vel = np.array(tcp_state[7])  # worldLinkAngularVelocity
        tcp_vel = np.array([*tcp_lin_vel, *tcp_ang_vel])

        return tcp_pose, tcp_vel

    def get_tcp_pose(self):
        """
        Returns pose of the Tool Center Point in world frame.
        """
        cur_TCP_pose, _ = self.get_current_tcp_pose_vel()
        return np.array(cur_TCP_pose)

    def get_tcp_vel(self):
        """
        Returns velocity of the Tool Center Point in world frame.
        """
        _, cur_TCP_vel = self.get_current_tcp_pose_vel()
        return np.array(cur_TCP_vel)

    def get_joint_angles(self):
        """
        Returns joint positions of the robot arm.
        """
        joint_pos, _ = self.get_current_joint_pos_vel()
        return np.array(joint_pos)

    def get_joint_vel(self):
        """
        Returns joint velocities of the robot arm.
        """
        _, joint_vel = self.get_current_joint_pos_vel()
        return np.array(joint_vel)

    def compute_gravity_compensation(self):
        """
        Calculates torques to apply that compensate for effect of gravity.
        """
        cur_joint_pos, cur_joint_vel = self.get_current_joint_pos_vel()
        grav_comp_torque = self._pb.calculateInverseDynamics(
            self.embodiment_id, cur_joint_pos, cur_joint_vel, [0] * self.num_control_dofs
        )
        return np.array(grav_comp_torque)

    def apply_gravity_compensation(self):
        """
        Applys motor torques that compensate for gravity.
        """
        grav_comp_torque = self.compute_gravity_compensation()

        self._pb.setJointMotorControlArray(
            bodyIndex=self.embodiment_id,
            jointIndices=self.control_joint_ids,
            controlMode=self._pb.TORQUE_CONTROL,
            forces=grav_comp_torque,
        )

    def set_target_tcp_pose(self, target_pose):
        """
        Go directly to a tcp position specified relative to the worldframe.
        """

        # transform from work_frame to world_frame
        target_pos, target_rpy = target_pose[:3], target_pose[3:]
        target_orn = np.array(self._pb.getQuaternionFromEuler(target_rpy))

        # get target joint poses through IK
        joint_positions = self._pb.calculateInverseKinematics(
            self.embodiment_id,
            self.tcp_link_id,
            target_pos,
            target_orn,
            restPoses=self.rest_poses,
            maxNumIterations=100,
            residualThreshold=1e-8,
        )
        joint_positions = self._process_ik_joint_positions(joint_positions)
        joint_velocities = np.array([0] * self.num_control_dofs)

        # set joint control
        self._pb.setJointMotorControlArray(
            self.embodiment_id,
            self.control_joint_ids,
            self._pb.POSITION_CONTROL,
            targetPositions=joint_positions,
            targetVelocities=joint_velocities,
            positionGains=[self.pos_gain] * self.num_control_dofs,
            velocityGains=[self.vel_gain] * self.num_control_dofs,
            forces=[self.max_force] * self.num_control_dofs,
        )

        # set target positions for blocking move
        self._target_joints_positions = np.array(joint_positions)

    def _process_ik_joint_positions(self, joint_positions):
        """Adapt a generic IK solution to an arm's joint constraints."""
        return np.asarray(joint_positions)

    def set_target_tcp_velocities(self, target_vels):
        """
        Set desired tcp velocity.
        """

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
        # be careful of singnularities and non square matrices
        # use pseudo-inverse when this is the case
        # this is all the time for 7 dof arms like panda
        if jac.shape[1] > np.linalg.matrix_rank(jac.T):
            inv_jac = np.linalg.pinv(jac)
        else:
            inv_jac = np.linalg.inv(jac)

        # convert desired velocities from cart space to joint space
        joint_vels = np.matmul(inv_jac, target_vels)

        # apply joint space velocities
        self._pb.setJointMotorControlArray(
            self.embodiment_id,
            self.control_joint_ids,
            self._pb.VELOCITY_CONTROL,
            targetVelocities=joint_vels,
            velocityGains=[self.vel_gain] * self.num_control_dofs,
            forces=[self.max_force] * self.num_control_dofs,
        )

        # set target positions for blocking move
        self._target_joints_velocities = np.array(joint_vels)

    def set_target_joint_positions(self, joint_positions):
        """
        Go directly to a specified joint configuration.
        """
        joint_velocities = np.array([0] * self.num_control_dofs)

        # set joint control
        self._pb.setJointMotorControlArray(
            self.embodiment_id,
            self.control_joint_ids,
            self._pb.POSITION_CONTROL,
            targetPositions=joint_positions,
            targetVelocities=joint_velocities,
            positionGains=[self.pos_gain] * self.num_control_dofs,
            velocityGains=[self.vel_gain] * self.num_control_dofs,
            forces=[self.max_force] * self.num_control_dofs,
        )

        # set target positions for blocking move
        self._target_joints_positions = np.array(joint_positions)

    def set_target_joint_velocities(self, joint_velocities):
        """
        Set the desired joint velicities.
        """
        self._pb.setJointMotorControlArray(
            self.embodiment_id,
            self.control_joint_ids,
            self._pb.VELOCITY_CONTROL,
            targetVelocities=joint_velocities,
            positionGains=[self.pos_gain] * self.num_control_dofs,
            velocityGains=[self.vel_gain] * self.num_control_dofs,
            forces=[self.max_force] * self.num_control_dofs,
        )

    def step_sim(self):
        """
        Take a step of the simulation whilst applying neccessary forces
        """

        # compensate for the effect of gravity
        self.apply_gravity_compensation()

        # step the simulation
        self._pb.stepSimulation()

    def set_constant_vel_percentage(self, percentage):
        """
        Sets constant velocity for position moves as a percentage of maximum.
        """
        if percentage == float("inf"):
            self._constant_vel = None
            self._max_position_move_steps = 1000
        else:
            if percentage < 1 or percentage > 100:
                raise Exception("Speed value outside range of 1-100%")

            constant_vel_range = self._max_constant_vel - self._min_constant_vel
            self._constant_vel = self._min_constant_vel + constant_vel_range * (percentage / 100.0)
            self._max_position_move_steps = 10000

    def get_constant_vel_percentage(self):
        """
        Gets constant velocity used for position moves as a percentage of maximum..
        """
        if self._constant_vel is None:
            return float("inf")
        else:
            constant_vel_range = self._max_constant_vel - self._min_constant_vel
            return (self._constant_vel - self._min_constant_vel) / constant_vel_range

    def blocking_position_move(
        self,
        max_steps=1000,
        constant_vel=None,
        j_pos_tol=0.1,
        j_vel_tol=0.1,
    ):
        """
        step the simulation until a target position has been reached or the max
        number of steps has been reached
        """
        # get target position
        targ_j_pos = self._target_joints_positions

        for i in range(max_steps):

            # get the current joint positions and velocities
            cur_j_pos, cur_j_vel = self.get_current_joint_pos_vel()

            # Move with constant velocity (from google-ravens)
            # break large position move to series of small position moves.
            if constant_vel is not None:
                diff_j = np.array(targ_j_pos) - np.array(cur_j_pos)
                norm = np.linalg.norm(diff_j)
                v = diff_j / norm if norm > 0 else np.zeros_like(cur_j_pos)
                step_j = cur_j_pos + v * constant_vel

                # reduce vel if joints are close enough,
                # this helps to acheive final pose
                if all(np.abs(diff_j) < constant_vel):
                    constant_vel /= 2

                # set joint control
                self._pb.setJointMotorControlArray(
                    self.embodiment_id,
                    self.control_joint_ids,
                    self._pb.POSITION_CONTROL,
                    targetPositions=step_j,
                    targetVelocities=[0.0] * self.num_control_dofs,
                    positionGains=[self.pos_gain] * self.num_control_dofs,
                    velocityGains=[self.vel_gain] * self.num_control_dofs
                )

            # step the simulation
            self.step_sim()

            # calc totoal velocity
            j_pos_err = np.sum(np.abs(targ_j_pos - cur_j_pos))
            j_vel_err = np.sum(np.abs(cur_j_vel))

            # break if the pose error is small enough
            # and the velocity is low enough
            if (j_pos_err < j_pos_tol) and (j_vel_err < j_vel_tol):
                break

        # Warn user is correct pose was not reached within max steps
        if i == max_steps-1:
            warnings.warn("Blocking position move failed to reach tolerance within max_steps.")

    def blocking_velocity_move(
        self,
        blocking_steps=100
    ):
        """
        step the simulation until a target position has been reached or the max
        number of steps has been reached
        """
        for i in range(blocking_steps):
            # step the simulation
            self.step_sim()

    def apply_blocking_velocity_move(self, blocking_steps):
        self.blocking_velocity_move(blocking_steps=blocking_steps)

    def apply_blocking_position_move(self, quick_mode=False):
        if not quick_mode:
            # slow but more realistic moves
            self.blocking_position_move(
                max_steps=self._max_position_move_steps,
                constant_vel=self._constant_vel,
                j_pos_tol=1e-6,
                j_vel_tol=1e-3,
            )

        else:
            # fast but unrealistic moves (bigger_moves = worse performance)
            self.blocking_position_move(
                max_steps=1000,
                constant_vel=None,
                j_pos_tol=1e-6,
                j_vel_tol=1e-3,
            )

    def move_linear(self, targ_pose, quick_mode=False):
        self.set_target_tcp_pose(targ_pose)
        self.apply_blocking_position_move(quick_mode=quick_mode)

    def move_linear_vel(self, targ_vels, blocking_steps=100):
        self.set_target_tcp_velocities(targ_vels)
        self.apply_blocking_velocity_move(blocking_steps=blocking_steps)

    def move_joints(self, targ_joint_angles, quick_mode=False):
        self.set_target_joint_positions(targ_joint_angles)
        self.apply_blocking_position_move(quick_mode=quick_mode)

    def move_joints_vel(self, targ_vels, blocking_steps=100):
        self.set_target_joint_velocities(targ_vels)
        self.apply_blocking_velocity_move(blocking_steps=blocking_steps)

    """
    ==================== Debug Tools ====================
    """

    def draw_ee(self, lifetime=0.1):
        draw_link_frame(self.embodiment_id, self.link_name_to_index["ee_link"], lifetime=lifetime)

    def draw_tcp(self, lifetime=0.1):
        draw_link_frame(self.embodiment_id, self.tcp_link_id, lifetime=lifetime)
