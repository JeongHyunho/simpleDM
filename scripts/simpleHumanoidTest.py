import time

import pybullet as p
import pybullet_data
import numpy as np
import os
import json
from gym_simpleHumanoidMimic.envs.simple_humanoid_indices import *
from env import humanoid_data


def drawLinkAngVel(p, body_id, IND, line_id, line_len=0.3, color=(0., 1., 0.), width=1.):
    link = p.getLinkState(body_id,
                           IND,
                           computeForwardKinematics=True,
                           computeLinkVelocity=True)
    link_xyz = np.array(link[LINK_WORLD_POS])
    ang_vel = np.array(link[LINK_WORLD_ANGVEL])
    if line_id == []:
        line_id = p.addUserDebugLine(link_xyz,
                                     link_xyz+line_len*ang_vel,
                                     lineColorRGB=color,
                                     lineWidth=width)
    else:
        line_id = p.addUserDebugLine(link_xyz,
                                     link_xyz+line_len*ang_vel,
                                     lineColorRGB=color,
                                     lineWidth=width,
                                     replaceItemUniqueId=line_id)
    return line_id


sub_id = 1
trial = 9
data_path = humanoid_data.get_data_path()

sample_file = open(os.path.join(data_path, 'simpleHumanoid3D_sub' + str(sub_id) \
                                + '_trial' + str(trial) + '.json'), 'r')
sample_data = json.load(sample_file)

physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.81)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

humanoid_flag = p.URDF_MAINTAIN_LINK_ORDER + p.URDF_USE_SELF_COLLISION + p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS

planeID = p.loadURDF("plane_implicit.urdf")
start_pos = sample_data['init_xyz']
start_orn = p.getQuaternionFromEuler([0, 0, 0])
humanoid_path = os.path.join(data_path, 'simpleHumanoid3D_sub' + str(sub_id) + '.urdf')
humanoid_id = p.loadURDF(humanoid_path, start_pos, start_orn, useFixedBase=True, flags=humanoid_flag)

for i in range(-1, NLINK):
    p.changeVisualShape(humanoid_id, i, rgbaColor=[1, 1, 1, 0.7])

nframe = sample_data['nframe']
frame_id = p.addUserDebugParameter("frame", 0, nframe, 0)

njoint = p.getNumJoints(humanoid_id)
for i in range(njoint):
    p.setJointMotorControlMultiDof(humanoid_id,
                                   i,
                                   p.POSITION_CONTROL,
                                   [0, 0, 0, 1],
                                   targetVelocity=[0, 0, 0],
                                   positionGain=0,
                                   velocityGain=1,
                                   force=[0, 0, 0])

link_keys = ['base', 'Rhip', 'Rknee', 'Rankle', 'Lhip', 'Lknee', 'Lankle']
target_orn = [[] for _ in range(NLINK)]
target_lvel = [[] for _ in range(NLINK)]
target_avel = [[] for _ in range(NLINK)]

target_frame = 0

RTHIGH_line_id = []
RSHANK_line_id = []
RFOOT_line_id = []

while (p.isConnected()):
    target_frame = p.readUserDebugParameter(frame_id)
    # if target_frame == nframe-1:
    #     target_frame = 0
    # else:
    #     target_frame += 1
    time.sleep(1/30)

    cur_frame = int(np.minimum(target_frame, nframe - 1))
    next_frame = int(np.minimum(cur_frame + 1, nframe - 1))
    frac = target_frame - cur_frame

    cur_basePos = np.array(sample_data['pos']['base'][cur_frame])
    next_basePos = np.array(sample_data['pos']['base'][next_frame])
    target_basePos = cur_basePos + frac * (next_basePos - cur_basePos)

    for ind in range(NLINK):
        cur_orn = sample_data['orn'][link_keys[ind]][cur_frame]
        cur_avel = np.array(sample_data['ang_vel'][link_keys[ind]][cur_frame])
        next_orn = sample_data['orn'][link_keys[ind]][next_frame]
        next_avel = np.array(sample_data['ang_vel'][link_keys[ind]][next_frame])
        if ind in JOINT_SPHERICAL:
            target_orn[ind] = p.getQuaternionSlerp(cur_orn, next_orn, frac)
        elif ind in JOINT_REVOLUTE:
            target_orn[ind] = cur_orn + frac * (next_orn - cur_orn)
        target_avel[ind] = cur_avel + frac * (next_avel - cur_avel)

    print("{}".format(target_basePos))

    p.resetBasePositionAndOrientation(humanoid_id, target_basePos, target_orn[BASE])
    p.resetBaseVelocity(humanoid_flag, target_lvel[BASE], target_avel[BASE])
    for ind in JOINT_SPHERICAL:
        p.resetJointStateMultiDof(humanoid_id,
                                  ind,
                                  targetValue=target_orn[ind],
                                  targetVelocity=target_avel[ind])
    for ind in JOINT_REVOLUTE:
        p.resetJointState(humanoid_id,
                          ind,
                          targetValue=target_orn[ind],
                          targetVelocity=target_avel[ind])

    # RTHIGH_line_id = drawLinkAngVel(p, humanoid_id, RTHIGH, RTHIGH_line_id)
    # RSHANK_line_id = drawLinkAngVel(p, humanoid_id, RSHANK, RSHANK_line_id)
    # RFOOT_line_id = drawLinkAngVel(p, humanoid_id, RFOOT, RFOOT_line_id)

    p.stepSimulation()



