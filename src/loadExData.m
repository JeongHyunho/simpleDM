function [sub_info, kinetics, kinematics] = loadExData(sub_num, trial_num)
% loading experiment data (marker, force plate)

fprintf('(%s) ', mfilename)
fprintf('Loading and processing a experiment data of subject %d, trial %d ... ', sub_num, trial_num)

sub_info = struct('id', sub_num, 'trial', trial_num, 'height', [], ...
    'mass', [], 'speed', []);
kinematics = struct('step_com', [], 'freq', [], 'cut_off', [], ...
    'time', [], 'marker_list', []);
kinetics = struct('step_GRF', [], 'step_cop', [], 'freq', [], ...
    'cut_off', [], 'HS', [], 'TO', []);

% set subject and trial number
if sub_num < 10
    sub_id = ['S00', num2str(sub_num)];
else
    sub_id = ['S0', num2str(sub_num)];
end

if trial_num < 10
    trial_id = ['T00', num2str(trial_num)];
else
    trial_id = ['T0', num2str(trial_num)];
end

% folder root from src and load subject information
root_dir = fileparts(fileparts(mfilename('fullpath')));
load(fullfile(root_dir, 'data', 'SubInfo'), 'SubInfo')

sub_index = find(strcmpi(SubInfo.subjN', sub_id));
sub_info.name = sub_id;
sub_info.height = SubInfo.Height(sub_index);
sub_info.mass = SubInfo.Mass(sub_index);
sub_info.speed = SubInfo.(trial_id)(sub_index);
sub_info.sex = SubInfo.Sex(sub_index);
sub_info.PlateID = SubInfo.PlateID{sub_index};

off_set = [SubInfo.Offset1{sub_index}, SubInfo.Offset2{sub_index}];

% set up for data processing
kinematics.cut_off = 15;
kinetics.cut_off = 30;

% load data from excel file
[cap_raw, cap_txt] = xlsread(fullfile(root_dir, 'motion_data', sub_id, ...
    [sub_id, '_', trial_id, '.trc.xls']));
[pla_raw, pla_txt] = xlsread(fullfile(root_dir, 'motion_data', sub_id, ...
    [sub_id, '_', trial_id, '.anc.xls']));

nMarkers = (size(cap_txt, 2) - 2)/3;
kinematics.time = cap_raw(7:end, 2);
kinematics.freq = round(1/mean(diff(kinematics.time)));
[b_cap, a_cap] = butter(3, 2*kinematics.cut_off/kinematics.freq);

kinetics.time = pla_raw(12:end, 1);
kinetics.freq = kinematics.freq;
r_deci = round(1/mean(diff(kinetics.time)))/kinematics.freq;

% filtering and storing
% 58 markers in total (13 virtual markers)
% (45) 'F_Head', 'R_Head', 'L_Head', 'C7', 'T10', 'CLAV', 'STRN', 'R_Shoulder', ...
% 'R_Upper', 'R_ElbowL', 'R_ElbowM', 'R_Fore', 'R_WristL', 'R_WristM', 'R_Hand', ...
% 'L_Shoulder', 'L_Upper', 'L_ElbowL', 'L_ElbowM', 'L_Fore', 'L_WristL', 'L_WristT', ...
% 'L_WristM', 'L_Hand', 'R_ASIS', 'L_ASIS', 'V_Sacral', 'R_Sacral', 'L_Sacral', ...
% 'R_Thigh', 'R_Knee', 'R_Shank', 'R_Ankle', 'R_Heel', 'R_Little', 'R_Toe', ...
% 'R_Thumb', 'L_Thigh', 'L_Knee', 'L_Shank', 'L_Ankle', 'L_Heel', 'L_Little', ...
% 'L_Toe', 'L_Thumb'
% (13) 'V_Mid_ASIS', 'V_Pelvis_Origin', 'V_R_Hip_JC', 'V_L_Hip_JC', 'V_R_Knee_JC', ...
% 'V_L_Knee_JC', 'V_R_Ankle_JC', 'V_L_Ankle_JC', 'V_Mid_Hip', 'V_R_Toe_Offset_Static', ...
% 'V_L_Toe_Offset_Static', 'V_R_Toe_Offset', 'V_L_Toe_Offset'
kinematics.marker_list = cell(nMarkers, 1);
for i = 1:nMarkers
    kinematics.marker_list{i} = strrep(cap_txt{4, 3*i}, '.','_');
end

for i = 1:nMarkers
    marker_name = kinematics.marker_list{i};
    kinematics.(marker_name).X = filtfilt(b_cap, a_cap, cap_raw(7:end, 3*i))/1e3;
    kinematics.(marker_name).Y = filtfilt(b_cap, a_cap, cap_raw(7:end, 3*i+1))/1e3;
    kinematics.(marker_name).Z = filtfilt(b_cap, a_cap, cap_raw(7:end, 3*i+2))/1e3;
end

sens = [0.5 -0.5 1 0.6 0.3 0.3 0.5 -0.5 1 0.6 0.3 0.3]*5000/2^11;
for i = 1:12
    force_name = pla_txt{9, i+1};
    force_deci = decimate(pla_raw(12:end, i+1), r_deci);
    kinetics.(force_name(1:2)).(force_name(3)) = ...
        filtfilt(b_cap, a_cap, force_deci*sens(i)) - off_set(i);
end

% find HS, TO, and leg of HS from force plate
if strcmp(trial_id, 'T009')
    [HS, TO] = findRunHSTO(kinetics, kinematics, sub_info.PlateID);
else
    [HS, TO, HS_leg] = findGaitHSTO(kinetics, kinematics);
    kinetics.HS_leg = HS_leg;
end

kinetics.HS = HS;
kinetics.TO = TO;

% find standing duration
if strcmp(trial_id, 'T009')
    stand_dur = findRunStandingDur(kinetics, sub_info.PlateID);
else
    stand_dur = findGaitStandingDur(kinetics);
end

kinetics.stand_dur = stand_dur;

fprintf('done!\n\n')
