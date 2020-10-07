import os
import warnings
import time

import matplotlib.pyplot as plt
import numpy as np

from stable_baselines.common.misc_util import set_global_seeds

from stable_baselines.common.cmd_util import make_vec_env

from stable_baselines.common.vec_env.dummy_vec_env import DummyVecEnv

from gym_simpleHumanoidMimic.envs import SimpleHumanoidMimicEnv
from src.custom_policies import NormalMlpPolicy
from src.custom_ppo2 import PPO2

import tensorflow as tf
tf.logging.set_verbosity(tf.logging.ERROR)


def make_env(rank, seed=0, sub_id=6, enable_draw=True):
    def _init():
        env = SimpleHumanoidMimicEnv(sub_id=sub_id, enable_draw=enable_draw)

        # Important: use a different seed for each environment
        env.seed(seed + rank)
        return env

    set_global_seeds(seed)
    return _init


def plot_reward_portions(fullfilename):
    with open(fullfilename, 'r') as f:
        step = []
        r_orn = []
        r_avel = []
        r_end = []
        r_com = []
        f.readline()    # ignore the first line
        for line in f:
            data = line.split()
            step.append(eval(data[0]))
            r_orn.append(eval(data[1]))
            r_avel.append(eval(data[2]))
            r_end.append(eval(data[3]))
            r_com.append(eval(data[4]))

    fig, ax = plt.subplots()
    lines = ax.plot(step, r_orn, step, r_avel, step, r_end, step, r_com)
    plt.xticks(np.linspace(0, 10**7, 5),
               ["{:.0f}M".format(x / 10**6) for x in np.linspace(0, 10**7, 5)])
    plt.legend(lines, ['orientation', 'angular vel', 'end effector', 'com'], loc=1)
    plt.setp(lines, linewidth=1.0, alpha=0.8)
    plt.xlabel('update')
    plt.ylabel('rewards')
    plt.show()
    pass



if __name__ == '__main__':
    os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
    warnings.simplefilter(action='ignore', category=FutureWarning)
    warnings.simplefilter(action='ignore', category=Warning)

    log_path = os.path.join('/', 'home', 'user', 'Dropbox', 'MATLAB_dropbox', 'DeepMimic', 'log')
    run_id = 'run_' + '10071436'
    run_file = run_id + '_simpleHumanoid.zip'

    plot_reward_portions(os.path.join(log_path, run_id, 'reward_portions.txt'))

    envs = DummyVecEnv([make_env(1)])
    model = PPO2.load(os.path.join(log_path, run_file), envs)

    obs = envs.reset()
    while True:
        action, _states = model.predict(obs)
        obs, rewards, dones, info = envs.step(action)
        envs.render()
        time.sleep(1/30)

    pass

