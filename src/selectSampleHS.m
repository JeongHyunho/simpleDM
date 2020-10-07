function [best_HS, best_ind] = selectSampleHS(HS, orn)
% find best HS in terms of periodicity of joints angles

fprintf('(%s) ', mfilename)
fprintf('Finding a best periodic stride, heel strike index ... ')

HS_errors = nan(numel(HS)-2, 1);

for i = 1:numel(HS)-2
    cur_quats = collectQuats(orn, HS(i));
    next_quats = collectQuats(orn, HS(i+2));
    
    HS_errors(i) = calQuatsErr(cur_quats, next_quats);
end

[best_HS, best_ind] = min(HS_errors);

fprintf('done!\n\n')
end

function quats = collectQuats(joints, i)
% quaternions of root, hip, knee, ankle (right, left)
quats = zeros(7, 4);
quats(1,:) = joints.base(i,:);
quats(2,:) = joints.Rhip(i,:);
quats(3,:) = joints.Rknee(i,:);
quats(4,:) = joints.Rankle(i,:);
quats(5,:) = joints.Lhip(i,:);
quats(6,:) = joints.Lknee(i,:);
quats(7,:) = joints.Lankle(i,:);
end

function err = calQuatsErr(q, p)
% calculate sum of differences of quaternion pairs
err = 0;
for i = 1:size(q,1)
    err = err + (1-q(i,:)*p(i,:)');
end
end
