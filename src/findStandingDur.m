function stand_dur = findStandingDur(kinetics)
% find indices for standing duration (start/ end)

stand_dur = [1, nan];
nframe = numel(kinetics.time);

% find the first toe off index
win_size = mean(diff(kinetics.HS))/2;
anal_range = 1:round(0.01*nframe);
LTO = findPlateEvent(kinetics.F1.Z(anal_range), 20, win_size, 'TO');
RTO = findPlateEvent(kinetics.F2.Z(anal_range), 20, win_size, 'TO');

stand_end = min([LTO; RTO]);
if isempty(stand_end)
    stand_end = anal_range(end);
end
stand_dur(2) = stand_end;

% insanity check
assert(std(kinetics.F1.Z(stand_dur)) < 0.05*mean(kinetics.F1.Z(stand_dur)) && ...
    std(kinetics.F2.Z(stand_dur)) < 0.05*mean(kinetics.F2.Z(stand_dur)), ...
    "GRF standard deviation is too large during upright pose")

end