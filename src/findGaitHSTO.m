function [HS, TO, HS_leg] = findGaitHSTO(kinetics, kinematics)
% find heel strike, toe off indices

% for upright-stand duration
latency = 10;

anal_range = (latency*kinematics.freq:...
    numel(kinematics.time)-latency*kinematics.freq)+1;
cap_data = kinematics.L_Heel.Z(anal_range);
[~, cap_HS] = findpeaks(-cap_data, ...
    'MinPeakHeight', -0.02*max(cap_data)-0.98*min(cap_data));
win_size = round(0.3*mean(diff(cap_HS)));
LHS = findPlateEvent(kinetics.F1.Z(anal_range), 20, win_size, 'HS')+latency*kinematics.freq-1;
LTO = findPlateEvent(kinetics.F1.Z(anal_range), 20, win_size, 'TO')+latency*kinematics.freq-1;
RHS = findPlateEvent(kinetics.F2.Z(anal_range), 20, win_size, 'HS')+latency*kinematics.freq-1;
RTO = findPlateEvent(kinetics.F2.Z(anal_range), 20, win_size, 'TO')+latency*kinematics.freq-1;

% remove redundant HS, TO
LTO(LTO < LHS(1)) = [];
RHS(RHS < LHS(1)) = [];
RTO(RTO < RHS(1)) = [];
RHS(RHS > RTO(end)) = [];
LHS(LHS > RHS(end)) = [];
HS = sort([RHS; LHS]);
TO = sort([RTO; LTO]);

% store which leg is for each HS
HS_leg = zeros(numel(HS), 1);
for i = 1:numel(HS)
    if any(HS(i) == RHS)
        HS_leg(i) = 'R';
    elseif any(HS(i) == LHS)
        HS_leg(i) = 'L';
    end
end

% (todo) check HS/TO of R/L sequence is allright