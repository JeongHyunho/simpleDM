import glob
import time
import os
import warnings
from datetime import datetime
from src.custom_callbacks import SaveGifCallback, DiscRwdTerminate
from src.custom_policies import NormalMlpPolicy
from src.custom_ppo2 import PPO2

from stable_baselines.common.callbacks import CallbackList
from stable_baselines.common.env_checker import check_env
from stable_baselines.common.cmd_util import make_vec_env

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
# env_vec = make_vec_env(lambda: env, n_envs=4)

learning_mode = True

if learning_mode:
    print('\n' + '='*16 + ' note ' + '='*16)
    print(run_id + ' large epochs test, action norm is on')
    print('nepochs: 12')
    print('=' * 38)

    net_arch = [dict(pi=[512, 256], vf=[512, 256])]
    init_obs_norm = env.obs_norm_params()

    policy_kwargs = dict(act_fun=tf.nn.relu,
                         net_arch=net_arch,
                         obs_norm_init=init_obs_norm)
    act_norm = True

    n_time_step = 15*10**6
    final_lr = 5.0 * 10 ** (-4)
    initial_lr = 1.0 * 10 ** (-3)

    save_gif_callback = SaveGifCallback(save_freq=int(0.5*10**6),
                                        save_path=os.path.join(log_path, run_id, 'traning_videos'),
                                        fps=int(1. / env._policy_step))
    rwd_term_callback = DiscRwdTerminate(th_perc=0.9,
                                         n_skip=500)
    callback = CallbackList([save_gif_callback, rwd_term_callback])

    model = PPO2(NormalMlpPolicy,
                 env,
                 gamma=0.95,
                 n_steps=2048,
                 nminibatches=4,
                 noptepochs=12,
                 learning_rate=lambda x: (1-x) * final_lr + x * initial_lr,
                 policy_kwargs=policy_kwargs,
                 act_norm=act_norm,
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

obs = env.reset()
for _ in range(500):
    action, _ = model.predict(obs)
    obs, reward, done, _ = env.step(action)

    if not enable_draw:
        env.render()
    else:
        time.sleep(env._policy_step)

    if done:
        obs = env.reset()


