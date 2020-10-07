import os
from collections import OrderedDict
from datetime import time
import numpy as np
import yaml
import tensorflow as tf
from stable_baselines.common.schedules import constfn

from stable_baselines import PPO2
from stable_baselines.common import set_global_seeds
from stable_baselines.common.vec_env import DummyVecEnv, VecNormalize
from utils import make_env, linear_schedule, get_wrapper_class
from utils.hyperparams_opt import hyperparam_optimization

env_id = 'gym_simpleHumanoidMimic:simpleHumanoidMimic-v0'
env_kwargs = {'sub_id':6, 'enable_draw':False}
root_path = os.path.join('/', 'home', 'user', 'Dropbox', 'MATLAB_dropbox', 'DeepMimic')

n_trials = 1000
n_jobs = 4
sampler = 'random'
pruner = 'median'

seed = np.random.randint(2**32 - 1)
set_global_seeds(seed)
tensorboard_log = os.path.join(root_path, 'log', env_id)

with open('ppo2.yml', 'r') as f:
    hyperparams_dict = yaml.safe_load(f)
    hyperparams = hyperparams_dict[env_id]

saved_hyperparams = OrderedDict([(key, hyperparams[key]) for key in sorted(hyperparams.keys())])

n_envs = hyperparams['n_envs']
n_timesteps = hyperparams['n_timesteps']
del hyperparams['n_envs']
del hyperparams['n_timesteps']

for key in ['learning_rate', 'cliprange', 'cliprange_vf']:
    if key not in hyperparams.keys():
        continue
    hyperparams[key] = constfn(float(hyperparams[key]))

if 'normalize' in hyperparams.keys():
    normalize = hyperparams['normalize']
    del hyperparams['normalize']
else:
    normalize = False

log_path = os.path.join('log', 'ppo2-tunning')
os.makedirs(log_path, exist_ok=True)
save_path = os.path.join(root_path, 'model')
params_path = "{}/{}".format(save_path, env_id)
os.makedirs(params_path, exist_ok=True)

env_wrapper = get_wrapper_class(hyperparams)
if 'env_wrapper' in hyperparams.keys():
    del hyperparams['env_wrapper']


def create_env(n_envs, eval_env=False, no_log=False):

    global hyperparams, env_kwargs
    log_dir = None if eval_env or no_log else save_path

    if n_envs == 1:
        env = DummyVecEnv([make_env(env_id,
                                    0,
                                    seed,
                                    wrapper_class=env_wrapper,
                                    log_dir=log_dir,
                                    env_kwargs=env_kwargs)])
    else:
        env = DummyVecEnv([make_env(env_id,
                                    wrapper_class=env_wrapper,
                                    log_dir=log_dir,
                                    env_kwargs=env_kwargs)])
        if normalize:
            local_normalize_kwargs = {'norm_reward': False}
            env = VecNormalize(env, **local_normalize_kwargs)

    return env


def create_model(*_arg, **kwargs):
    net_arch = [dict(pi=[512, 256], vf=[512, 256])]
    policy_kwargs = dict(act_fun=tf.nn.relu, net_arch=net_arch)
    return PPO2(env=create_env(n_envs, no_log=True),
                policy_kwargs=policy_kwargs,
                tensorboard_log=tensorboard_log,
                verbose=0,
                **kwargs)


data_frame = hyperparam_optimization('ppo2',
                                     create_model,
                                     create_env,
                                     n_trials=n_trials,
                                     hyperparams=hyperparams,
                                     n_jobs=n_jobs,
                                     seed=seed,
                                     sampler_method=sampler,
                                     pruner_method=pruner,
                                     verbose=True)

report_name = "report_{}_{}-trials-{}-{}-{}_{}.csv".format(env_id,
                                                           n_trials,
                                                           n_timesteps,
                                                           sampler,
                                                           pruner,
                                                           int(time.time()))
log_path = os.path.join(log_path, 'report', report_name)
os.makedirs(os.path.dirname(log_path), exist_ok=True)
data_frame.to_csv(log_path)