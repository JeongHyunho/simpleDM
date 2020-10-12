import os
from datetime import datetime

import tensorflow as tf
from stable_baselines.common.misc_util import set_global_seeds

from stable_baselines.common.vec_env.dummy_vec_env import DummyVecEnv

from gym_simpleHumanoidMimic.envs import SimpleHumanoidMimicEnv
from src.custom_callbacks import SaveGifCallback, DiscRwdTerminate, SaveRewardPortionsCallback, \
    SaveValuePortionsCallback
from src.custom_policies import NormalMlpPolicy
from src.custom_ppo2 import PPO2
from stable_baselines.common.callbacks import CallbackList

from stable_baselines.common import make_vec_env
from stable_baselines.common.env_checker import check_env


def make_env(rank, seed=0, sub_id=6, enable_draw=False):
    def _init():
        env = SimpleHumanoidMimicEnv(sub_id=sub_id, enable_draw=enable_draw)

        # Important: use a different seed for each environment
        env.seed(seed + rank)
        return env

    set_global_seeds(seed)
    return _init


log_path = os.path.join(*__file__.split('/')[:-2], 'log', 'run_' + datetime.now().strftime('%m%d%H%M'))

sub_id = 6
enable_draw = False
env = SimpleHumanoidMimicEnv(sub_id=sub_id, enable_draw=enable_draw)
check_env(env)

net_arch = [dict(pi=[512, 256], vf=[512, 256])]
obs_norm_init = env.obs_norm_params()
act_norm_init = env.act_norm_params()

# disable norm
# obs_norm_init = None
# act_norm_init = None
policy_kwargs = dict(act_fun=tf.nn.relu,
                     net_arch=net_arch,
                     obs_norm_init=obs_norm_init,
                     act_norm_init=act_norm_init)

n_time_step = 10 * 10 ** 6

model = PPO2(NormalMlpPolicy,
             env,
             gamma=0.95,
             n_steps=4096,
             nminibatches=4,
             noptepochs=4,
             learning_rate=5.0 * 10 ** (-4),
             policy_kwargs=policy_kwargs,
             verbose=False)

# model.save('test')
#
# del model
#
# # model = PPO2(NormalMlpPolicy, env)
# env = DummyVecEnv([make_env(1)])
# model = PPO2.load('test', env)
#
# pass

reward_callback = SaveRewardPortionsCallback(fullfilename='test1.txt')
value_callback = SaveValuePortionsCallback(fullfilename='test2.txt')
callback = CallbackList([reward_callback, value_callback])

model.learn(total_timesteps=n_time_step,
            log_interval=100,
            callback=callback)

