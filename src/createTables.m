% create anthropometric tables for human w/o head, arms
%
% Reference:
%
% De Leva, Paolo. "Adjustments to Zatsiorsky-Seluyanov's segment inertia
% parameters." Journal of biomechanics 29.9 (1996): 1223-1230.

% data directory
data_path = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
    'env', 'humanoid_data');

% mass, inertia table
antNames = {'mass', 'ry', 'rx', 'rz', 'com'};
segNames = {'trunk', 'thigh', 'shank', 'foot'};

tableFe = table([0.5824, 0.1478, 0.0481, 0.0129]', ...
    [0.357, 0.369, 0.271, 0.299]', ...
    [0.339, 0.364, 0.267, 0.279]', ...
    [0.171, 0.162, 0.093, 0.139]',...
    [0.4151, 0.3612, 0.4416, 0.4014]', ...
    'VariableNames', antNames, 'RowNames', segNames);

tableMa = table([0.6028, 0.1416, 0.0433, 0.0137]', ...
    [0.372, 0.329, 0.255, 0.257]', ...
    [0.347, 0.329, 0.249, 0.245]', ...
    [0.191, 0.149, 0.103, 0.124]',...
    [0.4486, 0.4095, 0.4459, 0.4415]', ...
    'VariableNames', antNames, 'RowNames', segNames);

antTable = struct('female', tableFe, 'male', tableMa);

% create subject specific segments info table
subNames = {'S001', 'S002', 'S003', 'S004', 'S005', 'S006', 'S007', 'S008', ...
    'S009', 'S010', 'S011', 'S012', 'S013', 'S014', 'S015', 'S016'};
segInfoMass = {'trunk mass', 'thigh mass', 'shank mass', 'foot mass'};
segInfoLen = {'trunk length mean', 'trunk length std', 'thigh length mean', ...
    'thigh length std', 'shank length mean', 'shank length std', ...
    'foot length mean', 'foot length std'};
segInfoCoM = {'trunk CoM', 'thigh CoM', 'shank CoM', 'foot CoM'};
tableSize = [numel(subNames), numel(segInfoMass)+numel(segInfoLen)+numel(segInfoCoM)];
segmentInfo = table('Size', tableSize, ...
    'VariableTypes', [repmat("double", 1, numel(segInfoMass)+numel(segInfoLen)), ...
    repmat("cell", 1, numel(segInfoCoM))], ...
    'VariableNames', [segInfoMass(:)', segInfoLen(:)', segInfoCoM(:)'], ...
    'RowNames', subNames);

% save tables
over_write = false;
if exist('simpleAntTable.mat', 'file') && ~over_write
    warning('simpleAntTable.mat file already exists')
else
    save(fullfile(data_path, 'simpleAntTable.mat'), 'antTable')
end

if exist('segmentInfo.mat', 'file') && ~over_write
    warning('segmentInfo.mat file already exists')
else
    save(fullfile(data_path, 'segmentInfo.mat'), 'segmentInfo')
end

    