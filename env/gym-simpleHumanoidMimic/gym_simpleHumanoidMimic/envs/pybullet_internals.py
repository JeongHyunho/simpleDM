import os
import random
import numpy as np

import pybullet as p
from pybullet_utils import bullet_client

from gym_simpleHumanoidMimic.envs.mocap_data import SimpleHumanoidMocap
from gym_simpleHumanoidMimic.envs.humanoid_pd import SimpleHumanoidPD
from gym_simpleHumanoidMimic.envs.simple_humanoid_indices import *
from env import humanoid_data

class PyBulletSimpleHumanoidMimicEnv:
    def __init__(self, sub_id, enable_draw):
        self._time_step = 1. / 240.
        self._sub_id = sub_id
        self._enable_draw = enable_draw
        self._timeout = 20
        self._fall_detect_body = [BASE, RTHIGH, RSHANK, LTHIGH, LSHANK]

        self._sim_time = 0
        self._cycle_count = 0
        self._phase = 0
        self._cur_frame = 0
        self._next_frame = 0

        # set-up pybullet simulation
        if self._enable_draw:
            self._pybullet_client = bullet_client.BulletClient(connection_mode=p.GUI)
        else:
            self._pybullet_client = bullet_client.BulletClient()
        self._pybullet_client.setTimeStep(self._time_step)
        self._pybullet_client.setGravity(0, 0, -9.8)

        # load motion capture data and pd
        data_path = humanoid_data.get_data_path()
        self._pybullet_client.setAdditionalSearchPath(data_path)
        mocap_path = os.path.join(data_path, 'simpleHumanoid3D_sub'+str(sub_id)+'_trial1.json')
        self._mocap = SimpleHumanoidMocap(self._pybullet_client,
                                          mocap_path)
        self._humanoid = SimpleHumanoidPD(self._pybullet_client,
                                          sub_id)

        # load plane
        self._planeId = self._pybullet_client.loadURDF("plane_implicit.urdf")
        self._pybullet_client.setPhysicsEngineParameter(numSolverIterations=10)
        self._pybullet_client.setPhysicsEngineParameter(numSubSteps=1)
        self._pybullet_client.changeDynamics(self._planeId, linkIndex=-1, lateralFriction=0.9)

        # reward parameters
        self.w_orn = 0.65
        self.w_avel = 0.1
        self.w_end = 0.15
        self.w_com = 0.1

        self.reset()

    def reset(self):
        start_time = float(random.randint(0, 1000)) / 1000 * self._mocap.cycle_time
        self._sim_time = start_time
        self.set_sim_time(start_time)

        # set pose to be in current phase
        self._mocap.reset_pose(self._humanoid.sim, self._phase)
        self._mocap.kin_update(self._humanoid.kin, self._phase, self._ncycle, reset=True)

        obs = self.get_obs()
        return obs

    def set_sim_time(self, time):
        self._sim_time = time
        self._phase = (time % self._mocap.cycle_time) / self._mocap.cycle_time
        self._ncycle = int(time / self._mocap.cycle_time)
        self._cur_frame = int(self._cur_frame % self._mocap.frame_dur)
        self._next_frame = np.min([self._cur_frame+1, self._mocap.nframe-1])

    def get_cycle_count(self, time):
        return np.ceil(self._sim_time / self._mocap.cycle_time)

    def get_obs(self):
        # state of simulation
        # consisting of phase(1), base z(1), pos(3*6), orn(4*7), linear vel(3*7), angular vel(3*7)
        # total: 1 + 1 + 18 + 28 + 21 + 21 = 90
        obs = np.zeros(90, dtype=np.float64)
        obs[0] = self._phase
        obs[1:] = self._humanoid.get_sim_state()
        return obs

    def update(self, action, nupdate):
        desired_pose = self._convert_to_pose(action)
        for _ in range(nupdate):
            self.set_sim_time(self._sim_time + self._time_step)
            self._humanoid.apply_pd_torque(desired_pose)
            self._pybullet_client.stepSimulation()
        self._mocap.kin_update(self._humanoid.kin, self._phase, self._ncycle, reset=False)

    def _convert_to_pose(self, action):
        # convert action to desired pose of humanoid
        pose = [[] for _ in range(1, NLINK)]
        action_ind = 0
        for link_ind, i in zip(range(1, NLINK), range(0, NLINK-1)):
            if link_ind in JOINT_SPHERICAL:
                # todo change q to euler for smaller params
                angle = action[action_ind]
                axis = [action[action_ind + 1], action[action_ind + 2], action[action_ind + 3]]
                pose_q = p.getQuaternionFromAxisAngle(axis, angle)
                pose[i] = [pose_q[0], pose_q[1], pose_q[2], pose_q[3]]
                action_ind += 4
            elif link_ind in JOINT_REVOLUTE:
                angle = action[action_ind]
                pose[i] = [angle]
                action_ind += 1
        return pose

    def cal_reward(self, obs, action):
        sim_links = self._pybullet_client.getLinkStates(self._humanoid.sim,
                                                        range(NLINK),
                                                        computeForwardKinematics=True,
                                                        computeLinkVelocity=True)
        kin_links = self._pybullet_client.getLinkStates(self._humanoid.kin,
                                                        range(NLINK),
                                                        computeForwardKinematics=True,
                                                        computeLinkVelocity=True)
        c_orn, c_avel, c_end, c_com = -2, -0.1, -40, -10

        sim_root = sim_links[BASE][LINK_WORLD_POS]
        kin_root = kin_links[BASE][LINK_WORLD_POS]
        sim_base_lvel = sim_links[BASE][LINK_WORLD_LINVEL]
        kin_base_lvel = kin_links[BASE][LINK_WORLD_LINVEL]
        sim_base_rel_orn = self._humanoid.orn_along_lin_vel(sim_base_lvel)
        kin_base_rel_orn = self._humanoid.orn_along_lin_vel(kin_base_lvel)
        sim_to_base_orn = [-sim_base_rel_orn[0],
                           -sim_base_rel_orn[1],
                           -sim_base_rel_orn[2],
                           -sim_base_rel_orn[3]]
        kin_to_base_orn = [-kin_base_rel_orn[0],
                           -kin_base_rel_orn[1],
                           -kin_base_rel_orn[2],
                           -kin_base_rel_orn[3]]

        orn_err_sqr = 0.
        avel_err_sqr = 0.
        sim_com = np.zeros(3)
        kin_com = np.zeros(3)
        end_err_sqr = 0.
        for link_ind in range(1, NLINK):
            # orientation reward
            sim_orn = sim_links[link_ind][LINK_LOCAL_ORN]
            kin_orn = kin_links[link_ind][LINK_LOCAL_ORN]
            if link_ind in JOINT_SPHERICAL:
                _, sim_rel_orn = self._pybullet_client.multiplyTransforms([0., 0., 0.],
                                                                          sim_to_base_orn,
                                                                          [0., 0., 0.],
                                                                          sim_orn)
                _, kin_rel_orn = self._pybullet_client.multiplyTransforms([0., 0., 0.],
                                                                          kin_to_base_orn,
                                                                          [0., 0., 0.],
                                                                          kin_orn)
                quat_diff = self._pybullet_client.getDifferenceQuaternion(sim_rel_orn, kin_rel_orn)
                _, ang_diff = self._pybullet_client.getAxisAngleFromQuaternion(quat_diff)
                orn_err_sqr += ang_diff * ang_diff
            elif link_ind in JOINT_REVOLUTE:
                ang_diff = kin_orn[0] - sim_orn[0]
                orn_err_sqr += ang_diff * ang_diff

            # angular velocity reward
            sim_avel = sim_links[link_ind][LINK_WORLD_ANGVEL]
            kin_avel = kin_links[link_ind][LINK_WORLD_ANGVEL]
            if link_ind in JOINT_SPHERICAL:
                sim_rel_avel = self._pybullet_client.rotateVector(sim_to_base_orn, sim_avel)
                kin_rel_avel = self._pybullet_client.rotateVector(kin_to_base_orn, kin_avel)
                avel_diff = [kin_rel_avel[0] - sim_rel_avel[0],
                             kin_rel_avel[1] - sim_rel_avel[1],
                             kin_rel_avel[2] - sim_rel_avel[2]]
                avel_err_sqr += avel_diff[0] * avel_diff[0] + avel_diff[1] * avel_diff[1] + avel_diff[2] * avel_diff[2]
            elif link_ind in JOINT_REVOLUTE:
                avel_diff = kin_avel[0] - sim_avel[0]
                avel_err_sqr += avel_diff * avel_diff

            # com reward
            sim_pos = np.array(sim_links[link_ind][LINK_WORLD_POS]) - np.array(sim_root)
            kin_pos = np.array(kin_links[link_ind][LINK_WORLD_POS]) - np.array(kin_root)
            sim_rel_pos = self._pybullet_client.rotateVector(sim_to_base_orn, sim_pos)
            kin_rel_pos = self._pybullet_client.rotateVector(kin_to_base_orn, kin_pos)
            sim_com += self._humanoid.link_mass[link_ind] / self._humanoid.mass * np.array(sim_rel_pos)
            kin_com += self._humanoid.link_mass[link_ind] / self._humanoid.mass * np.array(kin_rel_pos)

            # end effector reward
            if link_ind in self._humanoid.end_links:
                sim_rel_end = np.array(sim_rel_pos)
                kin_rel_end = np.array(kin_rel_pos)
                end_err_sqr += np.linalg.norm(sim_rel_end - kin_rel_end) ** 2

        com_err_sqr = np.linalg.norm(sim_com - kin_com)**2

        r_orn = self.w_orn * np.exp(c_orn * orn_err_sqr)
        r_avel = self.w_avel * np.exp(c_avel * avel_err_sqr)
        r_end = self.w_end * np.exp(c_end * end_err_sqr)
        r_com = self.w_com * np.exp(c_com * com_err_sqr)
        portions = [r_orn, r_avel, r_end, r_com]

        reward = r_orn + r_avel + r_end + r_com

        return reward, portions

    def is_end(self):
        # check if any non-allowed body part hits the ground (or time-out)
        done = False
        pts = self._pybullet_client.getContactPoints()
        for p in pts:
            if p[CONTACT_BODY_A] == p[CONTACT_BODY_B]:
                continue
            if p[CONTACT_BODY_A] == self._humanoid.sim:
                link_in_contact = p[CONTACT_LINK_A]
            elif p[CONTACT_BODY_B] == self._humanoid.sim:
                link_in_contact = p[CONTACT_LINK_B]
            if link_in_contact in self._fall_detect_body:
                done = True
        if self._sim_time > self._timeout:
            done = True
        return done

