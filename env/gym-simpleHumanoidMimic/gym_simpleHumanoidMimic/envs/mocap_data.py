import json
import numpy as np
from gym_simpleHumanoidMimic.envs.simple_humanoid_indices import *


class SimpleHumanoidMocap:
    def __init__(self, pybullet_client, mocap_path):
        self._pybullet_client = pybullet_client
        with open(mocap_path) as file_id:
            self._data = json.load(file_id)
        self.nframe = self._data['nframe']
        self.frame_dur = self._data['time'][1]
        self.cycle_time = self._data['time'][1] * (self.nframe - 1)

        self._base_period = self._data['pos']['base'][self.nframe - 1]
        self._base_shift = []

    def reset_pose(self, sim, phase):
        # set pose of sim based on data and phase
        cur_frame, next_frame, fraction = self._phase_to_frame(phase)
        pos, orn, lin_vel, ang_vel = self._lin_erp(cur_frame, next_frame, fraction)

        start_base_pos = [0., 0., pos[BASE][2]]
        self._pybullet_client.resetBasePositionAndOrientation(sim, start_base_pos, orn[BASE])
        self._pybullet_client.resetBaseVelocity(sim, lin_vel[BASE], ang_vel[BASE])

        self._set_pose_vel(sim, orn, ang_vel)

    def kin_update(self, kin, phase, ncycle, reset=False):
        # set pose of kin based on data and phase
        cur_frame, next_frame, fraction = self._phase_to_frame(phase)
        pos, orn, lin_vel, ang_vel = self._lin_erp(cur_frame, next_frame, fraction)

        if reset:
            self._base_shift = [-pos[BASE][0], -pos[BASE][1], -pos[BASE][2]]
            start_base_pos = [0., 0., pos[BASE][2]]
            self._pybullet_client.resetBasePositionAndOrientation(kin, start_base_pos, orn[BASE])
            self._pybullet_client.resetBaseVelocity(kin, lin_vel[BASE], ang_vel[BASE])
        else:
            start_base_pos = [self._base_period[0] * ncycle + self._base_shift[0] + pos[BASE][0],
                              self._base_period[1] * ncycle + self._base_shift[1] + pos[BASE][1],
                              pos[BASE][2]]
            self._pybullet_client.resetBasePositionAndOrientation(kin, start_base_pos, orn[BASE])
            self._pybullet_client.resetBaseVelocity(kin, lin_vel[BASE], ang_vel[BASE])

        self._set_pose_vel(kin, orn, ang_vel)

    def _phase_to_frame(self, phase):
        # convert phase to frame info
        cur_frame = int(phase * (self.nframe - 1) + 1)
        next_frame = np.min([cur_frame + 1, self.nframe-1])
        fraction = (phase % 1/(self.nframe - 1)) * (self.nframe - 1)
        return cur_frame, next_frame, fraction

    def _lin_erp(self, cur_frame, next_frame, fraction):
        pos = [[] for _ in range(NLINK)]
        orn = [[] for _ in range(NLINK)]
        lin_vel = [[] for _ in range(NLINK)]
        ang_vel = [[] for _ in range(NLINK)]

        for i in range(NLINK):
            cur_pos = self._data['pos'][JOINT_KEYS[i]][cur_frame]
            next_pos = self._data['pos'][JOINT_KEYS[i]][next_frame]
            pos[i] = [cur_pos[0] + fraction * (next_pos[0] - cur_pos[0]),
                      cur_pos[1] + fraction * (next_pos[1] - cur_pos[1]),
                      cur_pos[2] + fraction * (next_pos[2] - cur_pos[2])]

            cur_orn = self._data['orn'][JOINT_KEYS[i]][cur_frame]
            next_orn = self._data['orn'][JOINT_KEYS[i]][next_frame]
            if i in JOINT_SPHERICAL:
                orn[i] = self._pybullet_client.getQuaternionSlerp(cur_orn, next_orn, fraction)
            elif i in JOINT_REVOLUTE:
                orn[i] = cur_orn + fraction * (next_orn - cur_orn)

            cur_lin_vel = self._data['lin_vel'][JOINT_KEYS[i]][cur_frame]
            next_lin_vel = self._data['lin_vel'][JOINT_KEYS[i]][next_frame]
            lin_vel[i] = [cur_lin_vel[0] + fraction * (next_lin_vel[0] - cur_lin_vel[0]),
                          cur_lin_vel[1] + fraction * (next_lin_vel[1] - cur_lin_vel[1]),
                          cur_lin_vel[2] + fraction * (next_lin_vel[2] - cur_lin_vel[2])]

            cur_ang_vel = self._data['ang_vel'][JOINT_KEYS[i]][cur_frame]
            next_ang_vel = self._data['ang_vel'][JOINT_KEYS[i]][next_frame]
            if i in JOINT_SPHERICAL:
                ang_vel[i] = [cur_ang_vel[0] + fraction * (next_ang_vel[0] - cur_ang_vel[0]),
                              cur_ang_vel[1] + fraction * (next_ang_vel[1] - cur_ang_vel[1]),
                              cur_ang_vel[2] + fraction * (next_ang_vel[2] - cur_ang_vel[2])]
            elif i in JOINT_REVOLUTE:
                ang_vel[i] = cur_ang_vel + fraction * (next_ang_vel - cur_ang_vel)
        return pos, orn, lin_vel, ang_vel

    def _set_pose_vel(self, model, orn, ang_vel):
        # set all joints' orientation and angular velocity
        self._pybullet_client.resetJointStatesMultiDof(model,
                                                       JOINT_SPHERICAL[1:],
                                                       [orn[i] for i in JOINT_SPHERICAL[1:]],
                                                       [ang_vel[i] for i in JOINT_SPHERICAL[1:]])
        for rev_ind in JOINT_REVOLUTE:
            self._pybullet_client.resetJointState(model, rev_ind, orn[rev_ind], ang_vel[rev_ind])
