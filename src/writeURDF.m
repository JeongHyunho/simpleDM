function s_seg = writeURDF(HS_index, sub, kinetics, kinematics, samplePath, data_dir)
% write simple humanoid segments info in urdf, considering subject specific
% collision, inertia, joint (shape/ origin)


% set base and target urdf file
base_filename = "humanoid_simple.urdf";
txt_filename = "simpleHumanoid3D_sub"+sub.id+".urdf";
if nargin < 6
    env_dir = fileparts(fileparts(mfilename('fullpath')));
    data_dir = fullfile(env_dir, 'env', 'humanoid_data');
end
base_fullfile = fullfile(data_dir, base_filename);
txt_fullfile = fullfile(data_dir, txt_filename);

fprintf('(%s) ', mfilename)
fprintf('Writing simple humanoid segments info in urdf %s ... ', txt_fullfile)

% load anthropometric, segmentInfo table
load(fullfile(data_dir, 'simpleAntTable.mat'), 'antTable')
load(fullfile(data_dir, 'segmentInfo.mat'), 'segmentInfo')

if strcmp(sub.sex, 'F')
    bodyTable = antTable.female;
else
    bodyTable = antTable.male;
end

% calculate mass, length, inertia, joints / store collision
seg_list = ["trunk", "thigh", "shank", "foot"];
joint_list = ["Rhip", "Rknee", "Rankle", "Lhip", "Lknee", "Lankle"];

s_inertia = struct('x', [], 'y', [], 'z', [], 'Ixx', [], 'Iyy', [], 'Izz', []);
s_collision = struct('origin', struct('x', [], 'y', [], 'z', []), 'param', []);
s_info = struct('mass', [], 'length_mean', [], 'length_std', [], ...
    'inertia', s_inertia, 'collision', s_collision);
s_seg = struct('trunk', s_info, 'thigh', s_info, 'shank', s_info, ...
    'foot', s_info);

s_jo = struct('x', [], 'y', [], 'z', [], 'quat', [0, 0, 0, 1]);
s_joint = struct('Rhip', s_jo, 'Rknee', s_jo, 'Rankle', s_jo, ...
    'Lhip', s_jo, 'Lknee', s_jo, 'Lankle', s_jo);

% mass
s_seg.trunk.mass = sub.mass*bodyTable.mass(strcmp('trunk', bodyTable.Row));
s_seg.thigh.mass = sub.mass*bodyTable.mass(strcmp('thigh', bodyTable.Row));
s_seg.trunk.mass = sub.mass*bodyTable.mass(strcmp('trunk', bodyTable.Row));
s_seg.shank.mass = sub.mass*bodyTable.mass(strcmp('shank', bodyTable.Row));
s_seg.foot.mass = sub.mass*bodyTable.mass(strcmp('foot', bodyTable.Row));

% length
nframe = kinetics.HS(HS_index+1)-kinetics.HS(HS_index)+1;
trunk_vec = zeros(nframe, 1);
Rthigh_vec = zeros(nframe, 1);
Rshank_vec = zeros(nframe, 1);
RfootLen_vec = zeros(nframe, 1);
RfootWid_vec = zeros(nframe, 1);
RfootJC2Heel_vec = zeros(nframe, 1);
Lthigh_vec = zeros(nframe, 1);
Lshank_vec = zeros(nframe, 1);
LfootLen_vec = zeros(nframe, 1);
LfootWid_vec = zeros(nframe, 1);
LfootJC2Heel_vec = zeros(nframe, 1);

for i = 1:nframe
    trunk_vec(i) = markerVecLen({'V_R_Hip_JC', 'V_L_Hip_JC'}, {'C7', 'STRN'}, i);
    Rthigh_vec(i) = markerVecLen({'V_R_Hip_JC'}, {'V_R_Knee_JC'}, i);
    Rshank_vec(i) = markerVecLen({'V_R_Knee_JC'}, {'V_R_Ankle_JC'}, i);
    RfootLen_vec(i) = markerVecLen({'R_Heel'}, {'V_R_Toe_Offset'}, i);
    RfootWid_vec(i) = markerVecLen({'R_Little'}, {'R_Toe'}, i);
    RfootJC2Heel_vec(i) = JC2HeelLen('R', i);
    
    Lthigh_vec(i) = markerVecLen({'V_L_Hip_JC'}, {'V_L_Knee_JC'}, i);
    Lshank_vec(i) = markerVecLen({'V_L_Knee_JC'}, {'V_L_Ankle_JC'}, i);
    LfootLen_vec(i) = markerVecLen({'L_Heel'}, {'V_L_Toe_Offset'}, i);
    LfootWid_vec(i) = markerVecLen({'L_Little'}, {'L_Toe'}, i);
    LfootJC2Heel_vec(i) = JC2HeelLen('L', i);
end
RfootHei_vec = kinematics.R_Heel.Z(kinetics.stand_dur(1):kinetics.stand_dur(2));
LfootHei_vec = kinematics.L_Heel.Z(kinetics.stand_dur(1):kinetics.stand_dur(2));

s_seg.trunk.length_mean = mean(trunk_vec);
s_seg.trunk.length_std = std(trunk_vec);
s_seg.thigh.length_mean = mean([Rthigh_vec; Lthigh_vec]);
s_seg.thigh.length_std = std([Rthigh_vec; Lthigh_vec]);
s_seg.shank.length_mean = mean([Rshank_vec; Lshank_vec]);
s_seg.shank.length_std = std([Rshank_vec; Lshank_vec]);
s_seg.thigh.length_mean = mean([Rthigh_vec; Lthigh_vec]);
s_seg.thigh.length_std = std([Rthigh_vec; Lthigh_vec]);
s_seg.foot.length_mean = [mean([RfootLen_vec; LfootLen_vec]), ...
    mean([RfootWid_vec; LfootWid_vec]), ...
    mean([RfootHei_vec; LfootHei_vec]), ...
    mean([RfootJC2Heel_vec; LfootJC2Heel_vec])];
s_seg.foot.length_std = [std([RfootLen_vec; LfootLen_vec]), ...
    std([RfootWid_vec; LfootWid_vec]), ...
    std([RfootHei_vec; LfootHei_vec]), ...
    std([RfootJC2Heel_vec; LfootJC2Heel_vec])];

% inertia
for i = 1:numel(seg_list)
    ml2 = s_seg.(seg_list(i)).mass * s_seg.(seg_list(i)).length_mean(1) * ...
        [bodyTable.rx(strcmp(seg_list(i), bodyTable.Row)), ...
        bodyTable.ry(strcmp(seg_list(i), bodyTable.Row)), ...
        bodyTable.rz(strcmp(seg_list(i), bodyTable.Row))];
    s_seg.(seg_list(i)).inertia.Ixx = ml2(1);
    s_seg.(seg_list(i)).inertia.Iyy = ml2(2);
    s_seg.(seg_list(i)).inertia.Izz = ml2(3);
end

s_seg.trunk.inertia.x = 0;
s_seg.trunk.inertia.y = 0;
s_seg.trunk.inertia.z = (0.5 - bodyTable.com(strcmp('trunk', bodyTable.Row)))* ...
    s_seg.trunk.length_mean;
s_seg.thigh.inertia.x = 0;
s_seg.thigh.inertia.y = 0;
s_seg.thigh.inertia.z = -bodyTable.com(strcmp('thigh', bodyTable.Row))* ...
    s_seg.thigh.length_mean;
s_seg.shank.inertia.x = 0;
s_seg.shank.inertia.y = 0;
s_seg.shank.inertia.z = -bodyTable.com(strcmp('shank', bodyTable.Row))* ...
    s_seg.shank.length_mean;
s_seg.foot.inertia.x = 0;
s_seg.foot.inertia.y = bodyTable.com(strcmp('foot', bodyTable.Row))* ...
    s_seg.foot.length_mean(1) - s_seg.foot.length_mean(4);
s_seg.foot.inertia.z = -0.5*s_seg.foot.length_mean(3);

% collisions
s_seg.trunk.collision.origin.x = 0;
s_seg.trunk.collision.origin.y = 0;
s_seg.trunk.collision.origin.z = 0;
s_seg.thigh.collision.origin.x = 0;
s_seg.thigh.collision.origin.y = 0;
s_seg.thigh.collision.origin.z = -0.5*s_seg.thigh.length_mean;
s_seg.shank.collision.origin.x = 0;
s_seg.shank.collision.origin.y = 0;
s_seg.shank.collision.origin.z = -0.5*s_seg.shank.length_mean;
s_seg.foot.collision.origin.x = 0;
s_seg.foot.collision.origin.y = -s_seg.foot.length_mean(3)+0.5*s_seg.foot.length_mean(1);
s_seg.foot.collision.origin.z = -0.5*s_seg.foot.length_mean(3);

s_seg.trunk.collision.param = s_seg.trunk.length_mean*[0.8, 0.35, 1];
s_seg.thigh.collision.param = s_seg.thigh.length_mean*[0.2, 0.2, 1];
s_seg.shank.collision.param = s_seg.shank.length_mean*[0.2, 0.2, 1];
s_seg.foot.collision.param = [s_seg.foot.length_mean(2), ...
    s_seg.foot.length_mean(1), ...
    s_seg.foot.length_mean(3)];

% joints
hipJC_xLen_vec = zeros(nframe, 1);
for i = 1:nframe
    hipJC_xLen_vec(i) = markerVecLen({'V_R_Hip_JC'}, {'V_L_Hip_JC'}, i);
end
hipJC_xLen = mean(hipJC_xLen_vec);

s_joint.Rhip.x = 0.5*hipJC_xLen;
s_joint.Rhip.y = 0;
s_joint.Rhip.z = -0.5*s_seg.trunk.length_mean;
s_joint.Rknee.x = 0;
s_joint.Rknee.y = 0;
s_joint.Rknee.z = -s_seg.thigh.length_mean;
s_joint.Rankle.x = 0;
s_joint.Rankle.y = 0;
s_joint.Rankle.z = -s_seg.shank.length_mean;
s_joint.Lhip.x = -0.5*hipJC_xLen;
s_joint.Lhip.y = 0;
s_joint.Lhip.z = -0.5*s_seg.trunk.length_mean;
s_joint.Lknee.x = 0;
s_joint.Lknee.y = 0;
s_joint.Lknee.z = -s_seg.thigh.length_mean;
s_joint.Lankle.x = 0;
s_joint.Lankle.y = 0;
s_joint.Lankle.z = -s_seg.shank.length_mean;

% display seg info(mass/length/inertia) results for debug
fprintf('\n\t\t\t%s\t\t%s\t\t%s\t\t%s\n', 'Trunk', 'Thigh', 'Shank', 'Foot')
fprintf('mass (kg)\t%3.2f\t\t%3.2f\t\t%3.2f\t\t%3.2f\n', ...
    s_seg.trunk.mass, s_seg.thigh.mass, s_seg.shank.mass, s_seg.foot.mass)
fprintf('len_m (m)\t%3.2f\t\t%3.2f\t\t%3.2f\t\t%3.2f\n', s_seg.trunk.length_mean, ...
    s_seg.thigh.length_mean, s_seg.shank.length_mean, s_seg.foot.length_mean(1))
fprintf('len_s (m)\t%1.2E\t%1.2E\t%1.2E\t%1.2E\n', s_seg.trunk.length_std, ...
    s_seg.thigh.length_std, s_seg.shank.length_std, s_seg.foot.length_std(1))
fprintf('Ixx (kg*m2)\t%1.2E\t%1.2E\t%1.2E\t%1.2E\n', s_seg.trunk.inertia.Ixx, ...
    s_seg.thigh.inertia.Ixx, s_seg.shank.inertia.Ixx, s_seg.foot.inertia.Ixx)
fprintf('Iyy (kg*m2)\t%1.2E\t%1.2E\t%1.2E\t%1.2E\n', s_seg.trunk.inertia.Iyy, ...
    s_seg.thigh.inertia.Iyy, s_seg.shank.inertia.Iyy, s_seg.foot.inertia.Iyy)
fprintf('Izz (kg*m2)\t%1.2E\t%1.2E\t%1.2E\t%1.2E\n', s_seg.trunk.inertia.Izz, ...
    s_seg.thigh.inertia.Izz, s_seg.shank.inertia.Izz, s_seg.foot.inertia.Izz)
fprintf('com x (cm) \t%3.2f\t\t%3.2f\t\t%3.2f\t\t%3.2f\n', 100*s_seg.trunk.inertia.x, ...
    100*s_seg.thigh.inertia.x(1), 100*s_seg.shank.inertia.x(1), 100*s_seg.foot.inertia.x(1))
fprintf('com y (cm) \t%3.2f\t\t%3.2f\t\t%3.2f\t\t%3.2f\n', 100*s_seg.trunk.inertia.y, ...
    100*s_seg.thigh.inertia.y(1), 100*s_seg.shank.inertia.y(1), 100*s_seg.foot.inertia.y(1))
fprintf('com z (cm) \t%3.2f\t\t%3.2f\t\t%3.2f\t\t%3.2f\n', 100*s_seg.trunk.inertia.z, ...
    100*s_seg.thigh.inertia.z(1), 100*s_seg.shank.inertia.z(1), 100*s_seg.foot.inertia.z(1))


% save seg info in segmentInfo.mat
sub_ind = strcmp(sub.name, segmentInfo.Row);
segmentInfo.('trunk mass')(sub_ind) = s_seg.trunk.mass;
segmentInfo.('thigh mass')(sub_ind) = s_seg.thigh.mass;
segmentInfo.('shank mass')(sub_ind) = s_seg.shank.mass;
segmentInfo.('foot mass')(sub_ind) = s_seg.foot.mass;
segmentInfo.('trunk length mean')(sub_ind) = s_seg.trunk.length_mean;
segmentInfo.('thigh length mean')(sub_ind) = s_seg.thigh.length_mean;
segmentInfo.('shank length mean')(sub_ind) = s_seg.shank.length_mean;
segmentInfo.('foot length mean')(sub_ind) = s_seg.foot.length_mean(1);
segmentInfo.('trunk length std')(sub_ind) = s_seg.trunk.length_std;
segmentInfo.('thigh length std')(sub_ind) = s_seg.thigh.length_std;
segmentInfo.('shank length std')(sub_ind) = s_seg.shank.length_std;
segmentInfo.('foot length std')(sub_ind) = s_seg.foot.length_std(1);

save(fullfile(data_dir, 'segmentInfo.mat'), 'segmentInfo')

% fill in link(inertial, collision), joint(origin)/ collision
[humanoid, elem_seg, elem_joint] = humanoid2struct(base_fullfile);

link_list = ["trunk", "Rthigh", "Rshank", "Rfoot", "Lthigh", "Lshank", "Lfoot"];
link_match = ["trunk", "thigh", "shank", "foot", "thigh", "shank", "foot"];
for i = 1:numel(link_list)
    elem_seg.(link_list(i)).inertial.mass.setAttribute("value", num2str(s_seg.(link_match(i)).mass, "%.6f"));
    elem_seg.(link_list(i)).inertial.origin.setAttribute("xyz", ...
        sprintf("%.6f %.6f %.6f", [s_seg.(link_match(i)).inertia.x, ...
        s_seg.(link_match(i)).inertia.y, ...
        s_seg.(link_match(i)).inertia.z]));
    elem_seg.(link_list(i)).inertial.inertia.setAttribute("ixx", num2str(s_seg.(link_match(i)).inertia.Ixx, "%.6f"));
    elem_seg.(link_list(i)).inertial.inertia.setAttribute("iyy", num2str(s_seg.(link_match(i)).inertia.Iyy, "%.6f"));
    elem_seg.(link_list(i)).inertial.inertia.setAttribute("izz", num2str(s_seg.(link_match(i)).inertia.Izz, "%.6f"));
    
    elem_seg.(link_list(i)).collision.origin.setAttribute("xyz", ...
        sprintf("%.6f %.6f %.6f", ...
        [s_seg.(link_match(i)).collision.origin.x, ...
        s_seg.(link_match(i)).collision.origin.y, ...
        s_seg.(link_match(i)).collision.origin.z]))
    elem_seg.(link_list(i)).collision.box.setAttribute("size", ...
        sprintf("%.6f %.6f %.6f", s_seg.(link_match(i)).collision.param))
end

for i = 1:numel(joint_list)
    elem_joint.(joint_list(i)).origin.setAttribute("xyz", ...
        num2str([s_joint.(joint_list(i)).x, ...
        s_joint.(joint_list(i)).y, ...
        s_joint.(joint_list(i)).z], ' %.6f'));
end
elem_joint.Rknee.axis.setAttribute("xyz", "1.0 0.0 0.0")
elem_joint.Lknee.axis.setAttribute("xyz", "1.0 0.0 0.0")

% write revised urdf
xmlwrite(txt_fullfile, humanoid);

% write z-penetration at 0 frame
sample = jsondecode(fileread(samplePath));

base_xyz0 = sample.pos.base(1,:)';

RotmBase0 = quat2rotm(sample.orn.base(1,[4,1,2,3]));
RotmRhip0 = quat2rotm(sample.orn.Rhip(1,[4,1,2,3]));
RotmRknee0 = axang2rotm([1, 0, 0, sample.orn.Rknee(1)]);
RotmRankle0 = quat2rotm(sample.orn.Rankle(1,[4,1,2,3]));
RotmLhip0 = quat2rotm(sample.orn.Lhip(1,[4,1,2,3]));
RotmLknee0 = axang2rotm([1, 0, 0, sample.orn.Lknee(1)]);
RotmLankle0 = quat2rotm(sample.orn.Lankle(1,[4,1,2,3]));

r_trunk2Rhip = [0.5*hipJC_xLen, 0, -0.5*s_seg.trunk.length_mean]';
r_trunk2Lhip = [-0.5*hipJC_xLen, 0, -0.5*s_seg.trunk.length_mean]';
r_hip2knee = [0, 0, -s_seg.thigh.length_mean]';
r_knee2ankle = [0, 0, -s_seg.shank.length_mean]';

Rankle0 = base_xyz0+RotmBase0*(r_trunk2Rhip+RotmRhip0*(r_hip2knee+RotmRknee0*r_knee2ankle));
Lankle0 = base_xyz0+RotmBase0*(r_trunk2Lhip+RotmLhip0*(r_hip2knee+RotmLknee0*r_knee2ankle));

foot_dim_vecs = [0, 1, 0; 1, 0, 0; 0, 0, 1; 0, 1, 0]';
corner1 = foot_dim_vecs * diag([1, 0.5, -1, -1]) * s_seg.foot.length_mean';
corner2 = foot_dim_vecs * diag([1, -0.5, -1, -1]) * s_seg.foot.length_mean';
corner3 = foot_dim_vecs * diag([0, 0.5, -1, -1]) * s_seg.foot.length_mean';
corner4 = foot_dim_vecs * diag([0, -0.5, -1, -1]) * s_seg.foot.length_mean';

rel_corners = [corner1, corner2, corner3, corner4];
Rcorner = Rankle0 + RotmRankle0 * rel_corners;
Lcorner = Lankle0 + RotmLankle0 * rel_corners;

z_pen0 = min([Rcorner(3,:), Lcorner(3,:)]);
sample.pos.base(:,3) = sample.pos.base(:,3) - z_pen0;

% set initial z
init_xyz = zeros(1,3);
init_xyz(3) = s_seg.foot.length_mean(3) + s_seg.shank.length_mean+ ...
    s_seg.thigh.length_mean + 0.5*s_seg.trunk.length_mean;

sample.init_xyz = init_xyz;

fileID = fopen(samplePath, 'w');
sampleTxt = jsonencode(sample);
fprintf(fileID, '%s', sampleTxt);


fprintf('done!\n\n')


% ============ sub functions ============

    function l = markerVecLen(f1, f2, frame)
        % calculate distance between marker f1 and f2 in a frame
        
        vec1 = zeros(3,1);
        vec2 = zeros(3,1);
        for ii = 1:numel(f1)
            vec1 = vec1 + 1/numel(f1)*[kinematics.(f1{ii}).X(frame), ...
                kinematics.(f1{ii}).Y(frame), kinematics.(f1{ii}).Z(frame)]';
        end
        for jj = 1:numel(f2)
            vec2 = vec2 + 1/numel(f2)*[kinematics.(f2{jj}).X(frame), ...
                kinematics.(f2{jj}).Y(frame), kinematics.(f2{jj}).Z(frame)]';
        end
        l = norm(vec2 - vec1);
    end

    function l = JC2HeelLen(leg, frame)
        % calculate a length from heel to ankle joint projected to Y axis
        
        JC = "V_" + leg + "_Ankle_JC";
        heel =  leg + "_Heel";
        toe = "V_" + leg + "_Toe_Offset";
        
        vec1 = [kinematics.(JC).X(frame), kinematics.(JC).Y(frame), kinematics.(JC).Z(frame)]' - ...
            [kinematics.(heel).X(frame), kinematics.(heel).Y(frame), kinematics.(heel).Z(frame)]';
        vec2 = [kinematics.(toe).X(frame), kinematics.(toe).Y(frame), kinematics.(toe).Z(frame)]' - ...
            [kinematics.(heel).X(frame), kinematics.(heel).Y(frame), kinematics.(heel).Z(frame)]';
        
        l = vec1'*vec2/norm(vec2);   % projection to longitudinal axis
    end
end

function [humanoid, seg, joint] = humanoid2struct(fullfilename)
% converts simple humanoid urdf to struct

humanoid = xmlread(fullfilename);

link_name = ["base"; "trunk"; "Rthigh"; "Rshank"; "Rfoot"; "Lthigh"; "Lshank"; "Lfoot"];
joint_name = ["root"; "Rhip"; "Rknee"; "Rankle"; "Lhip"; "Lknee"; "Lankle"];

inertial_s = struct('origin', [], 'mass', [], 'inertia', []);
collision_s = struct('origin', [], 'box', []);
link_s = struct('inertial', inertial_s, 'collision', collision_s);
jo_s = struct('origin', [], 'limit', [], 'effort', []);

seg = struct('base', link_s, 'trunk', link_s, 'Rthigh', link_s, 'Rshank', link_s, ...
    'Rfoot', link_s, 'Lthigh', link_s, 'Lshank', link_s, 'Lfoot', link_s);
joint = struct('Rhip', jo_s, 'Rknee', jo_s, 'Rankle', jo_s, 'Lhip', jo_s, ...
    'Lknee', jo_s, 'Lankle', jo_s);

robot = humanoid.item(0);

% assign links (inertial/ collision)
item_seg = robot.getElementsByTagName('link');
for ii = 1:numel(link_name)
    seg.(link_name(ii)).inertial.origin = recurItem(item_seg.item(ii-1), ...
        ["inertial"; "origin"]);
    seg.(link_name(ii)).inertial.mass = recurItem(item_seg.item(ii-1), ...
        ["inertial"; "mass"]);
    seg.(link_name(ii)).inertial.inertia = recurItem(item_seg.item(ii-1), ...
        ["inertial"; "inertia"]);
    seg.(link_name(ii)).collision.origin = recurItem(item_seg.item(ii-1), ...
        ["collision"; "origin"]);
    seg.(link_name(ii)).collision.box = recurItem(item_seg.item(ii-1), ...
        ["collision"; "geometry"; "box"]);
end

% assign joints (origin, limit, axis)
item_joint = robot.getElementsByTagName('joint');
for ii = 1:numel(joint_name)
    joint.(joint_name(ii)).origin = recurItem(item_joint.item(ii-1), "origin");
    joint.(joint_name(ii)).limit = recurItem(item_joint.item(ii-1), "limit");
    joint.(joint_name(ii)).axis = recurItem(item_joint.item(ii-1), "axis");
end

    function child = recurItem(parent, names)
        for i = 1:numel(names)
            if ~isempty(parent)
                parent = parent.getElementsByTagName(names(i)).item(0);
            end
        end
        child = parent;
    end
end
