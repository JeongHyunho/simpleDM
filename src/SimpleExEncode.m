% process mocap data and make sample trajectory
% write simple humanoid segments info
sub_num = 6;
trial_num = 1;

% process mocap data
% write sample pos, GRF, CoM in joson
% write sample humanoid segments info in urdf

% (todo) seprate main codes and sub-functions

% load experiment data (subject information, kinetics data, kinematics data)
[sub_info, kinetics, kinematics] = loadExData(sub_num, trial_num);

% calculate quaternion of root, hip, knee, ankle (right/ left)
joints = simHumanoidJointsPose(sub_info, kinematics);

% find best HS in terms of periodicity of joints angles
[~, HS_index] = selectSampleHS(kinetics.HS, joints.orn);

% select one sample of time, joints, GRF, CoM
sample = selectSample(HS_index, sub_info, kinetics, joints);

% write sample data in json file
samplePath = writeSample(sub_info, sample);

% write simple humanoid segments info in urdf
writeURDF(HS_index, sub_info, kinetics, kinematics, samplePath);
