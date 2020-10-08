import os
import warnings
import time

from stable_baselines.common.misc_util import set_global_seeds
from stable_baselines.common.vec_env.dummy_vec_env import DummyVecEnv

from gym_simpleHumanoidMimic.envs import SimpleHumanoidMimicEnv
from src.custom_ppo2 import PPO2

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
warnings.simplefilter(action='ignore', category=FutureWarning)
warnings.simplefilter(action='ignore', category=Warning)
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


if __name__ == '__main__':
    log_path = os.path.join('/', 'home', 'user', 'Dropbox', 'MATLAB_dropbox', 'DeepMimic', 'log')
    run_id = 'run_' + '10080054'
    run_file = run_id + '_simpleHumanoid.zip'

    envs = DummyVecEnv([make_env(1)])
    model = PPO2.load(os.path.join(log_path, run_file), envs)

    obs = envs.reset()
    while True:
        action, _states = model.predict(obs)
        obs, rewards, dones, info = envs.step(action)
        envs.render()
        time.sleep(1/30)
    pass

