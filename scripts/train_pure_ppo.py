import glob
import os
import warnings
from datetime import datetime

from src.custom_callbacks import SaveGifCallback, DiscRwdTerminate
from stable_baselines.common.policies import MlpPolicy

from stable_baselines import PPO2
from stable_baselines.common.callbacks import CallbackList
from stable_baselines.common.env_checker import check_env

from gym_simpleHumanoidMimic.envs.simpleHumanoidMimic_env import SimpleHumanoidMimicEnv


os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
warnings.simplefilter(action='ignore', category=FutureWarning)
warnings.simplefilter(action='ignore', category=Warning)
import tensorflow as tf
tf.logging.set_verbosity(tf.logging.ERROR)

log_path = os.path.join('/', 'home', 'user', 'Dropbox', 'MATLAB_dropbox', 'DeepMimic', 'log')
run_id = 'run_' + datetime.now().strftime('%m%d%H%M')

sub_id = 6
enable_draw = False
env = SimpleHumanoidMimicEnv(sub_id=sub_id, enable_draw=enable_draw)
check_env(env)

learning_mode = True

if learning_mode:
    print('\n' + '='*16 + ' note ' + '='*16)
    print(run_id + ' pure ppo without any normalizer test')
    print('batch: 2048, nminibatches: 4')
    print('=' * 38)

    net_arch = [dict(pi=[512, 256], vf=[512, 256])]
    policy_kwargs = dict(act_fun=tf.nn.relu,
                         net_arch=net_arch,)

    n_time_step = 20*10**6
    final_lr = 5.0 * 10 ** (-4)
    initial_lr = 1.0 * 10 ** (-3)

    save_gif_callback = SaveGifCallback(save_freq=int(0.5*10**6),
                                        save_path=os.path.join(log_path, run_id, 'traning_videos'),
                                        fps=int(1. / env._policy_step))
    rwd_term_callback = DiscRwdTerminate(th_perc=0.9,
                                         n_skip=500)
    callback = CallbackList([save_gif_callback, rwd_term_callback])

    model = PPO2(MlpPolicy,
                 env,
                 gamma=0.95,
                 n_steps=2048,
                 nminibatches=4,
                 noptepochs=4,
                 learning_rate=lambda x: (1-x) * final_lr + x * initial_lr,
                 policy_kwargs=policy_kwargs,
                 verbose=False,
                 tensorboard_log=os.path.join(log_path, run_id))
    model.learn(total_timesteps=n_time_step,
                log_interval=100,
                reset_num_timesteps=False,
                tb_log_name='log_' + run_id,
                callback=callback)
    model.save(os.path.join(log_path, run_id + '_simpleHumanoid.zip'))
else:
    model_files = glob.glob(os.path.join(log_path, '*_simpleHumanoid'))
    assert not model_files, 'no model file have been saved'
    model_last = max(model_files, key=os.path.getctime)
    model = PPO2.load('*_simpleHumanoid')

