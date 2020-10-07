import numpy as np
from gym_simpleHumanoidMimic.envs.simpleHumanoidMimic_env import SimpleHumanoidMimicEnv
from gym_simpleHumanoidMimic.envs.simple_humanoid_indices import *


def draw_env_local_axis(env):
    if 'coord_id' not in globals():
        global coord_id
        coord_id = [x for x in range(NLINK * 3)]

    links = env._p.getLinkStates(env._internal_env._humanoid.sim, range(0, NLINK))

    for i in range(0, NLINK):
        link = links[i]
        pos = link[LINK_WORLD_POS]
        orn = link[LINK_WORLD_ORN]
        loc_x = env._p.rotateVector(orn, [1., 0., 0.])
        loc_y = env._p.rotateVector(orn, [0., 1., 0.])
        loc_z = env._p.rotateVector(orn, [0., 0., 1.])
        x_ind, y_ind, z_ind = 3*i, 3*i+1, 3*i+2
        coord_id[x_ind] = env._p.addUserDebugLine(pos,
                                                  np.array(pos) + 0.5 * np.array(loc_x),
                                                  [1., 0., 0.],
                                                  lineWidth=0.5,
                                                  lifeTime=0.,
                                                  replaceItemUniqueId=coord_id[x_ind])
        coord_id[y_ind] = env._p.addUserDebugLine(pos,
                                                  np.array(pos) + 0.5 * np.array(loc_y),
                                                  [0., 1., 0.],
                                                  lineWidth=0.5,
                                                  lifeTime=0.,
                                                  replaceItemUniqueId=coord_id[y_ind])
        coord_id[z_ind] = env._p.addUserDebugLine(pos,
                                                  np.array(pos) + 0.5 * np.array(loc_z),
                                                  [0., 0., 1.],
                                                  lineWidth=0.5,
                                                  lifeTime=0.,
                                                  replaceItemUniqueId=coord_id[z_ind])

sub_id = 6
enable_draw = True

env = SimpleHumanoidMimicEnv(sub_id=sub_id, enable_draw=enable_draw)

action = np.array([
    1. / 2., 1., 0., 0.,
    0.,
    0., 1., 0., 0.,
    0., 1., 0., 0.,
    1.,
    0., 1., 0., 0.,
])

obs = env.reset()
for _ in range(500):
    obs, reward, done, _ = env.step(action)
    draw_env_local_axis(env)

