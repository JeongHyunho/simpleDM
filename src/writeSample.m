function fullfilename = writeSample(sub, sample, data_dir)
% write sample data (pose, GRF, CoM) in txt file in data directory
% using pandas mudule, you could read json file in python

txt_filename = "simpleHumanoid3D_sub"+sub.id+"_trial"+sub.trial+".json";
if nargin < 3
    env_dir = fileparts(fileparts(mfilename('fullpath')));
    data_dir = fullfile(env_dir, 'env', 'humanoid_data');
end
fullfilename = fullfile(data_dir, txt_filename);
fileID = fopen(fullfilename, 'w');

fprintf('(%s) ', mfilename)
fprintf('Writing sample data in %s ... ', fullfilename)

% encode data in format of json, and write file
sample.nframe = numel(sample.time);
txt = jsonencode(sample);
fprintf(fileID, '%s', txt);

fprintf('done!\n\n')
end