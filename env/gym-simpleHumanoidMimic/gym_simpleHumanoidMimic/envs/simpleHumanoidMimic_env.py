import gym
from gym import error, spaces, utils
from gym.utils import seeding
import numpy as np
import cv2

from gym_simpleHumanoidMimic.envs.pybullet_internals import PyBulletSimpleHumanoidMimicEnv


class SimpleHumanoidMimicEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self, sub_id=6, enable_draw=True):
        self._sub_id = sub_id
        self._internal_env = PyBulletSimpleHumanoidMimicEnv(self._sub_id, enable_draw)
        self._p = self._internal_env._pybullet_client
        self._policy_step = 1. / 30.
        self._nupdate = int(self._policy_step / self._internal_env._time_step)
        self._img_size = (300, 200)

        self._act_max = np.array([np.pi, 1., 1., 1.,
                                  0.,
                                  np.pi, 1., 1., 1.,
                                  np.pi, 1., 1., 1.,
                                  0.,
                                  np.pi, 1., 1., 1.])
        self._act_min = np.array([0., -1., -1., -1.,
                                  -np.pi,
                                  0., -1., -1., -1.,
                                  0., -1., -1., -1.,
                                  -np.pi,
                                  0., -1., -1., -1.])
        action_max = np.array(self._act_max, dtype=np.float32)
        action_min = np.array(self._act_min, dtype=np.float32)
        self.action_space = spaces.Box(action_min, action_max)

        observation_max = np.array([np.inf] * 90, dtype=np.float32)
        observation_min = np.array([-np.inf] * 90, dtype=np.float32)
        self.observation_space = spaces.Box(observation_min, observation_max)

        self.seed()

    def step(self, action):
        self._internal_env.update(action, self._nupdate)

        reward, portions = self._internal_env.cal_reward
        obs = self._internal_env.get_obs()
        done = self._internal_env.is_end()
        info = {'portions': portions}
        return obs, reward, done, info

    def reset(self):
        state = self._internal_env.reset()
        return state

    def render(self, mode='human'):
        base_pos, _ = self._p.getBasePositionAndOrientation(self._internal_env._humanoid.sim)
        base_pos = [base_pos[0], base_pos[1], 0.7]

        view_mat = self._p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=base_pos,
                                                             distance=3,
                                                             yaw=90,
                                                             pitch=0.3,
                                                             roll=0,
                                                             upAxisIndex=2)
        proj_mat = self._p.computeProjectionMatrixFOV(fov=30,
                                                      aspect=self._img_size[0]/self._img_size[1],
                                                      nearVal=0.1,
                                                      farVal=100.)
        _, _, px, _, _ = self._p.getCameraImage(width=self._img_size[0],
                                                height=self._img_size[1],
                                                renderer=self._p.ER_BULLET_HARDWARE_OPENGL,
                                                viewMatrix=view_mat,
                                                projectionMatrix=proj_mat)

        rgb_array = np.reshape(np.array(px), (self._img_size[1], self._img_size[0], -1))
        bgr_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGBA2BGRA)
        if mode == 'human':
            if 'named_win' not in locals():
                named_win = cv2.namedWindow('simpleHumanoidMimic-v0')
            cv2.imshow('simpleHumanoidMimic-v0', rgb_array)
            cv2.waitKey(int(1000 * self._policy_step))
        else:
            return bgr_array

    def seed(self, seed=None):
        _, seed = seeding.np_random(seed)
        return [seed]

    # todo: update obs & act norm params to show results

    def obs_norm_params(self):
        # observation initial mean and std
        # phase(1), z(1), pos(3*6), orn(4*7), linear vel(3*7), angular vel(3*7)
        init_mean = np.array([0.5]*1 + [0.]*1 + [0.]*18 + [0.]*28 + [0.]*21 + [0.]*21)
        init_std = np.array([0.5]*1 + [1.]*1 + [1.]*18 + [1.]*28 + [1.]*21 + [1.]*21)
        return dict(init_mean=init_mean, init_std=init_std)

    def act_norm_params(self):
        # action initial mean and std
        # scale down action in range [min(a), max(a)] to [-1, 1] linearly
        init_mean = (self._act_max + self._act_min) / 2.
        init_std = (self._act_max - self._act_min) / 2.
        return dict(init_mean=init_mean, init_std=init_std)
