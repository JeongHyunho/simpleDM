import pybullet as p
import pybullet_data
import numpy as np
import os
import json
from gym_simpleHumanoidMimic.envs.simple_humanoid_indices import *
from env import humanoid_data

sub_id = 6
data_path = humanoid_data.get_data_path()

sample_file = open(os.path.join(data_path, 'simpleHumanoid3D_sub' + str(sub_id) + '_trial1.json'), 'r')
sample_data = json.load(sample_file)

physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.81)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

humanoid_flag = p.URDF_MAINTAIN_LINK_ORDER + p.URDF_USE_SELF_COLLISION + p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS

planeID = p.loadURDF("plane_implicit.urdf")
start_pos = sample_data['init_xyz']
start_orn = p.getQuaternionFromEuler([0, 0, 0])
humanoid_path = os.path.join(data_path,'simpleHumanoid3D_sub' + str(sub_id) + '.urdf')
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

joint_list = ['base', 'Rhip', 'Rknee', 'Rankle', 'Lhip', 'Lknee', 'Lankle']
spherical_ind = [1, 3, 4, 6]
revolute_ind = [2, 5]
target_orn = [[] for _ in range(NLINK)]

while (p.isConnected()):
    target_frame = p.readUserDebugParameter(frame_id)
    cur_frame = int(np.minimum(target_frame, nframe - 1))
    next_frame = int(np.minimum(cur_frame + 1, nframe - 1))
    frac = target_frame - cur_frame

    cur_basePos = np.array(sample_data['pos']['base'][cur_frame])
    next_basePos = np.array(sample_data['pos']['base'][next_frame])
    target_basePos = cur_basePos + frac * (next_basePos - cur_basePos)

    for i in range(len(joint_list)):
        cur_orn = sample_data['orn'][joint_list[i]][cur_frame]
        next_orn = sample_data['orn'][joint_list[i]][next_frame]
        if i in JOINT_SPHERICAL:
            target_orn[i] = p.getQuaternionSlerp(cur_orn, next_orn, frac)
        elif i in JOINT_REVOLUTE:
            target_orn[i] = cur_orn + frac * (next_orn - cur_orn)

    print("{}".format(target_basePos))

    p.resetBasePositionAndOrientation(humanoid_id, target_basePos, target_orn[0])
    for i in spherical_ind:
        p.resetJointStateMultiDof(humanoid_id, i, targetValue=target_orn[i])
    for i in revolute_ind:
        th = target_orn[i]
        p.resetJointState(humanoid_id, i, targetValue=th)

    p.stepSimulation()

