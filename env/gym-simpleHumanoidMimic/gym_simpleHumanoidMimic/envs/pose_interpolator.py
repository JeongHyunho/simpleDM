import numpy as np
from gym_simpleHumanoidMimic.envs.simple_humanoid_indices import *


class PoseInterpolator:
    def __init__(self):
        self._basePos = []
        self._jointOrn = []
        self._jointOrnVel = []
        self.reset()

    def reset(self):
        self._basePos = np.array([0., 0., 0.])
        # Base, Rthigh, Rshank, Rfoot, Lthigh, Lshank, Lfoot
        self._jointOrn = [[0., 0., 0., 1.],
                          [0., 0., 0., 1.],
                          [1.],
                          [0., 0., 0., 1.],
                          [0., 0., 0., 1.],
                          [1.],
                          [0., 0., 0., 1.]]
        self._jointOrnVel = [[0., 0., 0.],
                             [0., 0., 0.],
                             [0.],
                             [0., 0., 0.],
                             [0., 0., 0.],
                             [0.],
                             [0., 0., 0.]]

    def get_joint_position(self, indices):
        pos = []
        for i in range(len(indices)):
            pos += self._jointOrn[indices[i]]
        return np.concatenate(pos)

    def get_joint_velocity(self, indices):
        vel = []
        for i in range(len(indices)):
            vel += self._jointOrn[indices[i]]
        return np.concatenate(vel)

