
NLINK = 7

BASE = 0
RTHIGH = 1
RSHANK = 2
RFOOT = 3
LTHIGH = 4
LSHANK = 5
LFOOT = 6

# indices for getLinkStates
LINK_WORLD_POS = 0
LINK_WORLD_ORN = 1
LINK_LOCAL_POS = 2
LINK_LOCAL_ORN = 3
LINK_FRAME_POS = 4
LINK_FRAME_ORN = 5
LINK_WORLD_LINVEL = 6
LINK_WORLD_ANGVEL = 7

# indices for getJointStates
JOINT_POS = 0
JOINT_VEL = 1
JOINT_FORCE = 2
JOINT_APP_TORQUE = 3

JOINT_SPHERICAL= [BASE, RTHIGH, RFOOT, LTHIGH, LFOOT]
JOINT_REVOLUTE = [RSHANK, LSHANK]

JOINT_KEYS = ['base', 'Rhip', 'Rknee', 'Rankle', 'Lhip', 'Lknee', 'Lankle']

CONTACT_BODY_A = 1
CONTACT_BODY_B = 2
CONTACT_LINK_A = 3
CONTACT_LINK_B = 4
CONTACT_POS_A = 5
CONTACT_POS_B = 6

# indices for getDynamicsInfo method
DYN_MASS = 0
DYN_LAT_FRICTION = 1

# obervation of pos
OBS_POS_INDS = range(2, 2+3*6)

# observation indices
OBS_NAME = [[] for _ in range(90)]
OBS_NAME[2:2 + 3 * 6] = ['Obs_RThigh_x',
                         'Obs_RThigh_y',
                         'Obs_RThigh_z',
                         'Obs_RShank_x',
                         'Obs_RShank_y',
                         'Obs_RShank_z',
                         'Obs_RFoot_x',
                         'Obs_RFoot_y',
                         'Obs_RFoot_z',
                         'Obs_LThigh_x',
                         'Obs_LThigh_y',
                         'Obs_LThigh_z',
                         'Obs_LShank_x',
                         'Obs_LShank_y',
                         'Obs_LShank_z',
                         'Obs_LFoot_x',
                         'Obs_LFoot_y',
                         'Obs_LFoot_z']

# action indices
ACTIONS_NAME = ['Act_RThigh_angle',
                'Act_RThigh_x',
                'Act_RThigh_y',
                'Act_RThigh_z',
                'Act_RShank_angle',
                'Act_RShank_x',
                'Act_RShank_y',
                'Act_RShank_z',
                'Act_RFoot_angle',
                'Act_RFoot_x',
                'Act_RFoot_y',
                'Act_RFoot_z',
                'Act_LThigh_angle',
                'Act_LThigh_x',
                'Act_LThigh_y',
                'Act_LThigh_z',
                'Act_LShank_angle',
                'Act_LShank_x',
                'Act_LShank_y',
                'Act_LShank_z',
                'Act_LFoot_angle',
                'Act_LFoot_x',
                'Act_LFoot_y',
                'Act_LFoot_z']


