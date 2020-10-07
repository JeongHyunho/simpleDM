import numpy as np
from gym_simpleHumanoidMimic.envs.simple_humanoid_indices import *


class SimpleHumanoidPD:
    def __init__(self, pybullet_client, sub_id):
        self._pybullet_client = pybullet_client

        # load urdf (plane, humanoid for mocap & sim)
        flags = self._pybullet_client.URDF_MAINTAIN_LINK_ORDER + \
                self._pybullet_client.URDF_USE_SELF_COLLISION + \
                self._pybullet_client.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS + \
                self._pybullet_client.URDF_USE_INERTIA_FROM_FILE

        urdf_path = "simpleHumanoid3D_sub" + str(sub_id) + ".urdf"
        self.sim = self._pybullet_client.loadURDF(urdf_path,
                                                  basePosition=[0., 0., 2.],
                                                  useFixedBase=False,
                                                  flags=flags)
        self.kin = self._pybullet_client.loadURDF(urdf_path,
                                                  useFixedBase=True,
                                                  flags=self._pybullet_client.URDF_MAINTAIN_LINK_ORDER)
        self.end_links = [RFOOT, LFOOT]
        self.link_mass = []
        self.mass = 0.
        for i in range(NLINK):
            info = self._pybullet_client.getDynamicsInfo(self.sim, i)
            self.link_mass.append(info[DYN_MASS])
            self.mass += info[DYN_MASS]

        for i in range(-1, NLINK):
            self._pybullet_client.changeDynamics(self.sim, i, lateralFriction=0.9)

        self._pybullet_client.changeDynamics(self.sim, -1, linearDamping=0, angularDamping=0)
        self._pybullet_client.changeDynamics(self.kin, -1, linearDamping=0, angularDamping=0)

        for i in range(-1, NLINK):
            self._pybullet_client.setCollisionFilterGroupMask(self.kin,
                                                              i,
                                                              collisionFilterGroup=0,
                                                              collisionFilterMask=0)
            self._pybullet_client.changeDynamics(
                self.kin,
                i,
                activationState=self._pybullet_client.ACTIVATION_STATE_SLEEP
                                + self._pybullet_client.ACTIVATION_STATE_ENABLE_SLEEPING
                                + self._pybullet_client.ACTIVATION_STATE_DISABLE_WAKEUP)
            self._pybullet_client.changeVisualShape(self.kin, i, rgbaColor=[1, 1, 1, 0.4])

        # disable default constraint-based motors
        for i in range(0, NLINK):
            if i in JOINT_REVOLUTE:
                self._pybullet_client.setJointMotorControl2(self.sim,
                                                            i,
                                                            self._pybullet_client.VELOCITY_CONTROL,
                                                            targetVelocity=0,
                                                            force=0)
            elif i in JOINT_SPHERICAL:
                self._pybullet_client.setJointMotorControlMultiDof(self.sim,
                                                                   i,
                                                                   self._pybullet_client.POSITION_CONTROL,
                                                                   [0, 0, 0, 1],
                                                                   targetVelocity=[0, 0, 0],
                                                                   positionGain=0,
                                                                   velocityGain=1,
                                                                   force=[0, 0, 0])

        # setup for pd controller
        # parameter for thigh, shank, foot (R/L)
        self._max_forces = [3*[200], [150], 3*[90], 3*[200], [150], 3*[90]]
        self._kps = [500, 400, 300, 500, 400, 300]
        self._kds = [50, 40, 30, 50, 40, 30]
        self._target_vel = [3*[0.], [0.], 3*[0.], 3*[0.], [0.], 3*[0.]]

    def get_sim_state(self):
        # base z(1), pos(3*6), orn(4*7), linear vel(3*7), angular vel(3*7): totally 89
        states = []
        links_state = self._pybullet_client.getLinkStates(self.sim,
                                                          range(0, NLINK),
                                                          computeForwardKinematics=True,
                                                          computeLinkVelocity=True)

        base_pos = links_state[BASE][LINK_WORLD_POS]
        base_orn = links_state[BASE][LINK_WORLD_ORN]
        base_lin_vel = links_state[BASE][LINK_WORLD_LINVEL]

        rel_base_orn = self._orn_along_lin_vel(base_lin_vel)
        to_base_aligned = [-rel_base_orn[0], -rel_base_orn[1], -rel_base_orn[2], rel_base_orn[3]]

        # base z(1)
        base_z = base_pos[2]
        states.append(base_z)

        # pos(3*6)
        for i in range(1, NLINK):
            link_pos = links_state[i][LINK_WORLD_POS]
            rel_pos = [link_pos[0] - base_pos[0], link_pos[1] - base_pos[1], link_pos[2] - base_pos[2]]
            aligned_rel_pos = self._pybullet_client.rotateVector(to_base_aligned, rel_pos)
            for elem in aligned_rel_pos:
                states.append(elem)

        # base of orn(4*7)
        if base_orn[3] < 0:
            base_orn = [-base_orn[0], -base_orn[1], -base_orn[2], -base_orn[3]]
        for elem in base_orn:
            states.append(elem)

        # others of orn(4*7)
        for i in range(1, NLINK):
            link_orn = links_state[i][LINK_WORLD_ORN]
            _, link_rel_orn = self._pybullet_client.multiplyTransforms([0., 0., 0.],
                                                                       to_base_aligned,
                                                                       [0., 0., 0.],
                                                                       link_orn)
            if link_rel_orn[3] < 0:
                link_rel_orn = [-link_rel_orn[0], -link_rel_orn[1], -link_rel_orn[2], -link_rel_orn[3]]
            for elem in link_rel_orn:
                states.append(elem)

        # lin_vel(3*7)
        for i in range(NLINK):
            link_lin_vel = links_state[i][LINK_WORLD_LINVEL]
            rel_lin_vel = self._pybullet_client.rotateVector(to_base_aligned, link_lin_vel)
            for elem in rel_lin_vel:
                states.append(elem)

        # ang_vel(3*7)
        for i in range(NLINK):
            link_ang_vel = links_state[i][LINK_WORLD_ANGVEL]
            rel_ang_vel = self._pybullet_client.rotateVector(to_base_aligned, link_ang_vel)
            for elem in rel_ang_vel:
                states.append(elem)
        return states

    def apply_pd_torque(self, target_pose):
        # pose: thigh, shank, foot (R/L)
        self._pybullet_client.setJointMotorControlMultiDofArray(self.sim,
                                                                range(1, NLINK),
                                                                self._pybullet_client.STABLE_PD_CONTROL,
                                                                targetPositions=target_pose,
                                                                targetVelocities=self._target_vel,
                                                                forces=self._max_forces,
                                                                positionGains=self._kps,
                                                                velocityGains=self._kds)

    def _orn_along_lin_vel(self, lin_vel):
        # orn with x axis along lin_vel and z axis vertical to ground

        ux = np.array([lin_vel[0], lin_vel[1], 0.])
        uy = ux / np.linalg.norm(ux)
        uz = np.array([0., 0., 1.])
        ux = np.cross(uy, uz)
        orn_mat = np.c_[ux, uy, uz]
        qw = 0.5 * np.sqrt(1.0 + (orn_mat[0, 0] + orn_mat[1, 1] + orn_mat[2, 2]))
        qx = 0.
        qy = 0.
        qz = 0.5 * np.sign(orn_mat[1, 0] - orn_mat[0, 1]) * \
            np.sqrt(1.0 + (- orn_mat[0, 0] - orn_mat[1, 1] + orn_mat[2, 2]))

        return [qx, qy, qz, qw]
