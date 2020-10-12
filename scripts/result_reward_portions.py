import os
import warnings
import time

import cv2
import matplotlib.pyplot as plt
import numpy as np

import utils.plot
from stable_baselines.common.misc_util import set_global_seeds

from stable_baselines.common.cmd_util import make_vec_env

from stable_baselines.common.vec_env.dummy_vec_env import DummyVecEnv

from gym_simpleHumanoidMimic.envs import SimpleHumanoidMimicEnv
from src.custom_policies import NormalMlpPolicy
from src.custom_ppo2 import PPO2

import tensorflow as tf
tf.logging.set_verbosity(tf.logging.ERROR)


if __name__ == '__main__':
    os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
    warnings.simplefilter(action='ignore', category=FutureWarning)
    warnings.simplefilter(action='ignore', category=Warning)

    log_path = os.path.join(os.path.dirname(__file__)+'/..', 'log')
    run_id = 'run_' + '10102212'
    run_file = run_id + '_simpleHumanoid.zip'

    envs = DummyVecEnv([utils.make_env(1)])
    model = PPO2.load(os.path.join(log_path, run_file), envs)

    params = [envs.envs[0]._internal_env.w_orn,
              envs.envs[0]._internal_env.w_avel,
              envs.envs[0]._internal_env.w_end,
              envs.envs[0]._internal_env.w_com]

    # total_time_step = 20 * 10 ** 6
    # utils.plot.reward_portions(os.path.join(log_path, run_id, 'reward_portions.txt'),
    #                      params,
    #                      total_time_step)

    img = envs.render(mode='rgb_array')
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    writer = cv2.VideoWriter(run_id+'.avi', fourcc, 30.0, (img.shape[1], img.shape[0]))

    n_iter = 0
    obs = envs.reset()
    while n_iter < 500:
        action, _states = model.predict(obs, deterministic=True)
        obs, rewards, dones, info = envs.step(action)
        envs.render()
        img = envs.render(mode='rgb_array')
        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        writer.write(img_bgr)
        # time.sleep(1/30)      # sleep during render()
        n_iter += 1
    pass

    writer.release()

