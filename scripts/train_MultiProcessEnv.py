import os
import warnings
from datetime import datetime

import tensorflow.compat.v1 as tf

from gym_simpleHumanoidMimic.envs import SimpleHumanoidMimicEnv
from src.custom_callbacks import SaveGifCallback, DiscRwdTerminate
from src.custom_policies import NormalMlpPolicy
from src.custom_ppo2 import PPO2

from stable_baselines.common import set_global_seeds
from stable_baselines.common.callbacks import CallbackList

from stable_baselines.common.vec_env import SubprocVecEnv

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
warnings.simplefilter(action='ignore', category=FutureWarning)
warnings.simplefilter(action='ignore', category=Warning)

import tensorflow.compat.v1 as tf
tf.logging.set_verbosity(tf.logging.ERROR)

log_path = os.path.join('/', 'home', 'user', 'Dropbox', 'MATLAB_dropbox', 'DeepMimic', 'log')
run_id = 'run_' + datetime.now().strftime('%m%d%H%M')

print('\n' + '='*16 + ' note ' + '='*16)
print(run_id + ' obs act norm test')
print('obs normalized, action un-normalized')
print('action norm init param revised')
print('actor: [512, 256], critic: [512, 256]')
print('n_steps: 8192')
print('multiprocess, n = 4')
print('=' * 38)


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
    env_vec = SubprocVecEnv([make_env(i) for i in range(4)], start_method='spawn')

    net_arch = [dict(pi=[512, 256], vf=[512, 256])]
    obs_norm_init = env_vec.env_method('obs_norm_params', indices=0)[0]
    act_norm_init = env_vec.env_method('act_norm_params', indices=0)[0]
    policy_kwargs = dict(act_fun=tf.nn.relu,
                         net_arch=net_arch,
                         obs_norm_init=obs_norm_init,
                         act_norm_init=act_norm_init)

    save_gif_callback = SaveGifCallback(save_freq=int(0.5 * 10 ** 6),
                                        save_path=os.path.join(log_path, run_id, 'training_videos'),
                                        fps=int(1. / 30.))
    rwd_term_callback = DiscRwdTerminate(th_perc=0.9,
                                         n_skip=500)
    callback = CallbackList([save_gif_callback, rwd_term_callback])

    n_time_step = 10 ** 5

    model = PPO2(NormalMlpPolicy,
                 env_vec,
                 gamma=0.95,
                 n_steps=8192,
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
