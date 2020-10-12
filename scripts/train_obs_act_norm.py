import glob
import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import warnings
from datetime import datetime
import numpy as np

import gym
from src.custom_callbacks import SaveGifCallback, DiscRwdTerminate, SaveRewardPortionsCallback
from src.custom_policies import NormalMlpPolicy
from src.custom_ppo2 import PPO2

from stable_baselines.common.callbacks import CallbackList
from stable_baselines.common.env_checker import check_env

from gym_simpleHumanoidMimic.envs.simpleHumanoidMimic_env import SimpleHumanoidMimicEnv

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
warnings.simplefilter(action='ignore', category=FutureWarning)
warnings.simplefilter(action='ignore', category=Warning)
import tensorflow.compat.v1 as tf
tf.logging.set_verbosity(tf.logging.ERROR)

log_path = os.path.join('/', 'home', 'user', 'Dropbox', 'MATLAB_dropbox', 'DeepMimic', 'log')
run_id = 'run_' + datetime.now().strftime('%m%d%H%M')

sub_id = 6
trial = 1
enable_draw = False
env = SimpleHumanoidMimicEnv(sub_id=sub_id, trial=trial, enable_draw=enable_draw)

check_env(env)

print('\n' + '='*16 + ' note ' + '='*16)
print(run_id + ' obs act norm test')
print('obs normalized, action normalized')
print('action norm init param revised')
print('actor: [512, 256], critic: [512, 256]')
print('n_steps: 2**13')
print('entropy_c: 0.01 * 1/100')
print('reward portions recording')
print('reward_cal revised (joint revisit)')
print('ang vel fixed')
print('=' * 38)

net_arch = [dict(pi=[512, 256], vf=[512, 256])]
init_obs_norm = env.obs_norm_params()
init_act_norm = env.act_norm_params()
policy_kwargs = dict(act_fun=tf.nn.relu,
                     net_arch=net_arch,
                     obs_norm_init=init_obs_norm,
                     act_norm_init=init_act_norm)

n_time_step = 20*10**6

save_gif_callback = SaveGifCallback(save_freq=int(0.5*10**6),
                                    save_path=os.path.join(log_path, run_id, 'training_videos'),
                                    fps=int(1. / env._policy_step))
rwd_term_callback = DiscRwdTerminate(th_perc=0.9,
                                     n_skip=500)
rwd_rec_callback = SaveRewardPortionsCallback(fullfilename=os.path.join(log_path, run_id, 'reward_portions.txt'))

callback = CallbackList([save_gif_callback, rwd_term_callback, rwd_rec_callback])

model = PPO2(NormalMlpPolicy,
             env,
             gamma=0.95,
             n_steps=2**13,
             ent_coef=0.01*1/100,
             nminibatches=4,
             noptepochs=4,
             learning_rate=5.0 * 10 ** (-4),
             policy_kwargs=policy_kwargs,
             verbose=False,
             tensorboard_log=os.path.join(log_path, run_id))
model.learn(total_timesteps=n_time_step,
            log_interval=1000,
            reset_num_timesteps=False,
            tb_log_name='log_' + run_id,
            callback=callback)
model.save(os.path.join(log_path, run_id + '_simpleHumanoid.zip'))

