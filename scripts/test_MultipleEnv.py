import os
from datetime import datetime

import tensorflow.compat.v1 as tf

from gym_simpleHumanoidMimic.envs import SimpleHumanoidMimicEnv
from src.custom_callbacks import SaveGifCallback, DiscRwdTerminate
from src.custom_policies import NormalMlpPolicy
from src.custom_ppo2 import PPO2

from stable_baselines.common import set_global_seeds
from stable_baselines.common.callbacks import CallbackList

from stable_baselines.common.cmd_util import make_vec_env
from stable_baselines.common.env_checker import check_env
from stable_baselines.common.vec_env import SubprocVecEnv


def make_env(rank, seed=0):
    def _init():
        sub_id = 6
        enable_draw = False
        env = SimpleHumanoidMimicEnv(sub_id=sub_id, enable_draw=enable_draw)

        # Important: use a different seed for each environment
        env.seed(seed + rank)
        return env

    set_global_seeds(seed)
    return _init


if __name__ == '__main__':
    log_path = os.path.join(*__file__.split('/')[:-2], 'log', 'run_' + datetime.now().strftime('%m%d%H%M'))

    env_vec = SubprocVecEnv([make_env(i) for i in range(4)], start_method='spawn')

    net_arch = [dict(pi=[512, 256], vf=[512, 256])]
    obs_norm_init = env_vec.env_method('obs_norm_params', indices=0)[0]
    act_norm_init = env_vec.env_method('act_norm_params', indices=0)[0]
    policy_kwargs = dict(act_fun=tf.nn.relu,
                         net_arch=net_arch,
                         obs_norm_init=obs_norm_init,
                         act_norm_init=act_norm_init)

    n_time_step = 10 ** 5

    model = PPO2(NormalMlpPolicy,
                 env_vec,
                 gamma=0.95,
                 n_steps=8192,
                 nminibatches=4,
                 noptepochs=4,
                 learning_rate=5.0 * 10 ** (-4),
                 policy_kwargs=policy_kwargs,
                 verbose=False)

    model.learn(total_timesteps=n_time_step,
                log_interval=100)
