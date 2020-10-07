function [subject, capture, plate] = walkingEncode(fileName, cutOffFreq, ...
    modeSave, viz)

    switch nargin
        case 1
            cutOffFreq = [10 15];   % default cut-off frequency
            modeSave = true;
            viz = false;
        case 2
            modeSave = true;
            viz = false;
        case 3
            viz = false;
    end
    if viz, close all, end

    % fill in subject data
    Info = strsplit(fileName,'_');
    subject.Name = Info{1};
    subject.WalkingSpeed = Info{2};
    subject.Trial = Info{3};

    % save name, height, walking speed in 'subject'
    load([fileparts(fileparts(mfilename('fullpath'))) '/' 'SubInfo'], 'SubInfo')
    subLgc = strcmp(subject.Name, SubInfo.Name);
    if ~any(subLgc), error('No name in SubInfo.'); end
    height = SubInfo.Height(subLgc)*0.01;
    subject.Height = height;
    subject.mVel = SubInfo.(subject.WalkingSpeed)(subLgc);
    
    % initialize and load capture xls data
    capture = struct('StepTime',[],'StepCoM',[],'Freq',[],'CutOffFreq',[]);
    captureRaw = struct();
    [capRaw, capTxt] = xlsread([fileName '.trc.xls']);

    % capture data filtering
    markerNum = (size(capTxt, 2) - 2)/3;
    capture.Time = capRaw(7:end, 2);
    captureRaw.Time = capRaw(7:end, 2);
    capture.Freq = round(1/mean(capture.Time(2:end) - capture.Time(1:end-1)));
    [bCapture, aCapture] = butter(3, cutOffFreq(1)/(capture.Freq/2));
    capture.CutOffFreq = cutOffFreq(1);
    
    markerName = {'RHead', 'LHead', 'C7', 'RShoulder', 'LShoulder', ...
        'REllbow', 'LEllbow', 'RWrist', 'LWrist', 'RSacral', 'LSacral', ...
        'RHip', 'LHip', 'RAsis', 'LAsis', 'RKnee', 'LKnee', 'RAnkle', 'LAnkle', ...
        'RToe', 'LToe', 'RHeel', 'LHeel', 'Ref1', 'Ref2', 'Ref3', 'Ref4'};
    
    for i = 1:markerNum
        capture.(markerName{i}).X = ...
            filtfilt(bCapture, aCapture, capRaw(7:end, 3*i))/1e3;
        capture.(markerName{i}).Y = ...
            filtfilt(bCapture, aCapture, capRaw(7:end, 3*i+1))/1e3;
        capture.(markerName{i}).Z = ...
            filtfilt(bCapture, aCapture, capRaw(7:end, 3*i+2))/1e3;
        
        captureRaw.(markerName{i}).X = capRaw(7:end, 3*i)/1e3;
        captureRaw.(markerName{i}).Y = capRaw(7:end, 3*i+1)/1e3;
        captureRaw.(markerName{i}).Z = capRaw(7:end, 3*i+2)/1e3;
    end
    
    % initialize and load flate data
    plate = struct('StepTime',[],'StepGRF',[],'Freq',[],'CutOffFreq',[]);
    plateRaw = struct();
    [forceRaw, ~] = xlsread([fileName '.anc.xls']);
    g = 9.81;
    
    % down-sample force data to have same freq with capture data
    timeRaw = forceRaw(12:end,1 );
    plateFreq = round(1/mean(timeRaw(2:end) - timeRaw(1:end-1)));
    freqRatio = plateFreq/capture.Freq;
    for i = 1:13
        forceDeci(:,i) = decimate(forceRaw(12:end, i),freqRatio);
    end
    
    % plate data filtering
    plate.Time =capture.Time;
    plateRaw.Time = timeRaw;
    plate.Freq = plateFreq/freqRatio;
    [bForce, aForce] = butter(5, cutOffFreq(2)/(plate.Freq/2));
    plate.CutOffFreq = cutOffFreq(2);

    forceName = {'F1X', 'F1Y', 'F1Z', 'M1X', 'M1Y', 'M1Z', ...
        'F2X', 'F2Y', 'F2Z', 'M2X', 'M2Y', 'M2Z'};
    
    sens = [0.5 -0.5 1 0.6 0.3 0.3 0.5 -0.5 1 0.6 0.3 0.3]*5000/2^15;
    for i = 1:12
        plate.(forceName{i}(1:2)).(forceName{i}(3)) = ...
            filtfilt(bForce, aForce, forceDeci(:, i+1)*sens(i));
        plateRaw.(forceName{i}(1:2)).(forceName{i}(3)) = ...
            forceRaw(12:end, i+1)*sens(i);
    end
    subject.Mass = mean(plate.F1.Z(1:round(end/10))+plate.F2.Z(1:round(end/10)))/g;
    
    % for capture and plate sync, find time difference
    analDur = [30, 50]; % duration of analysis (sec)
    capIdx = analDur(1)*capture.Freq:analDur(2)*capture.Freq-1;
    plaIdx = analDur(1)*plate.Freq:analDur(2)*plate.Freq-1;
    
    lDelay = cycleDelay(captureRaw.LHeel.Z(capIdx), plate.F1.Z(plaIdx), ...
        capture.Freq, plate.Freq);
    rDelay = cycleDelay(captureRaw.RHeel.Z(capIdx), plate.F2.Z(plaIdx), ...
        capture.Freq, plate.Freq);
    
    if all([lDelay rDelay] > 10)
        warning('(%s) Delay detected. Mean Delay : %2.2f sec \n', ...
            fileName, (lDelay+rDelay)/2/capture.Freq)
    end
    assert(abs(lDelay-rDelay) <= 20, ...
        sprintf(['Detected right and left delays are too different.', ...
        'Right: %2.2f, Left %2.2f'], rDelay/capture.Freq, lDelay/capture.Freq))
    
    delayIdx = round((lDelay+rDelay)/2);
    
    if delayIdx >= 0
        capRange = 1:numel(capture.Time)-delayIdx;
        plaRange = 1+delayIdx:numel(plate.Time);
        plaRawRange = 1+delayIdx:numel(plate.Time);
    else
        capRange = 1-delayIdx:numel(capture.Time);
        plaRange = 1:numel(plate.Time)+delayIdx;
    end
    
    % for sync, change capture and plate data's range
    capture.Time = capture.Time(capRange)-capture.Time(capRange(1));
    plate.Time = plate.Time(plaRange)-plate.Time(plaRange(1));
    
    for i = 1:numel(markerName)
        capture.(markerName{i}).X = capture.(markerName{i}).X(capRange);
        capture.(markerName{i}).Y = capture.(markerName{i}).Y(capRange);
        capture.(markerName{i}).Z = capture.(markerName{i}).Z(capRange);
    end
    
    for i = 1:3:10
        plate.(forceName{i}(1:2)).X = plate.(forceName{i}(1:2)).X(plaRange);
        plate.(forceName{i}(1:2)).Y = plate.(forceName{i}(1:2)).Y(plaRange);
        plate.(forceName{i}(1:2)).Z = plate.(forceName{i}(1:2)).Z(plaRange);
    end    
    
    % find left and right leg's HS, TO index
    latency = 10;    % sec
    analRange = latency*plate.Freq:numel(plate.Time);
    capData =  captureRaw.LHeel.Z(capIdx);
    [~, capHS] = findpeaks(-capData, ...
        'MinPeakHeight', -0.02*max(capData)-0.98*min(capData));
    winSize = round(0.8*mean(diff(capHS)));
    LHS = findPlateEvent(plate.F1.Z(analRange), 20, winSize, 'HS')+latency*plate.Freq-1;
    LTO = findPlateEvent(plate.F1.Z(analRange), 20, winSize, 'TO')+latency*plate.Freq-1;
    RHS = findPlateEvent(plate.F2.Z(analRange), 20, winSize, 'HS')+latency*plate.Freq-1;
    RTO = findPlateEvent(plate.F2.Z(analRange), 20, winSize, 'TO')+latency*plate.Freq-1;
    
    mLen = min([numel(LHS), numel(LTO)]);
    assert(or(all(LHS(1:mLen) > LTO(1:mLen)), all(LHS(1:mLen) < LTO(1:mLen))), ...
        'LHS or LTO missing.')
    mLen = min([numel(RHS), numel(RTO)]);
    assert(or(all(RHS(1:mLen) > RTO(1:mLen)), all(RHS(1:mLen) < RTO(1:mLen))), ...
        'RHS or RTO missing.')
    
    % set start LHS index for save N strindes
    analStep = 10;
    [startLHS, ~]  = findStartIdx(plate.F1.Z,LHS,LTO,analStep,1e-1,viz); % find similar sequence's start index 

    fprintf('\n %s Start LHS : %d \n\n', fileName, startLHS)
    subject.AnalStep = analStep;
    subject.StartHS = startLHS;
    
    % From one of heel strikes, organize 10 steps'data (thesis -> start at 15th hs)
    %  --> 1step : LHS - RTO - RHS - LTO - LHS
    LHS = LHS(startLHS:startLHS+analStep/2+1);
    LTO(LTO < LHS(1)) = []; LTO = LTO(1:analStep/2+1);
    RHS(RHS < LHS(1)) = []; RHS = RHS(1:analStep/2+1);
    RTO(RTO < LHS(1)) = []; RTO = RTO(1:analStep/2+1);
    
    HS = sort([RHS; LHS]);
    TO = sort([RTO; LTO]);
    
    plate.HS = HS; plate.TO = TO;
    plate.RHS = RHS; plate.LHS = LHS;
    
    % Sacrum-stepSac, GRF-stepGRF
    SacZ = (capture.RSacral.Z + capture.LSacral.Z)/2;
    
    [capture.hhTime, capture.sacCoM.Z, capture.sacVel.Z] = ...
        sac2CoM(capture.Time, SacZ, plate.HS, plate.TO, analStep, viz);
    [capture.m2mTime, capture.m2mSac.Z, capture.m2mVel.Z] = ...
        sac2mm(capture.Time, SacZ, plate.HS, plate.TO, analStep, viz);
    
    plate.stepTime = cell(analStep,1);
    plate.stepGRF = struct('X',cell(analStep,1),'Y',cell(analStep,1),'Z',cell(analStep,1));
    for i = 1:analStep
        grfRange = HS(i):TO(i+1);
        plate.stepTime{i} = plate.Time(grfRange) - plate.Time(grfRange(1));
        if any(RHS == HS(i))
            plate.stepGRF(i).X = plate.F2.X(grfRange);
            plate.stepGRF(i).Y = plate.F2.Y(grfRange);
            plate.stepGRF(i).Z = plate.F2.Z(grfRange);
        elseif any(LHS == HS(i))
            plate.stepGRF(i).X = plate.F1.X(grfRange);
            plate.stepGRF(i).Y = plate.F1.Y(grfRange);
            plate.stepGRF(i).Z = plate.F1.Z(grfRange);
        else
            error('HS index in step is not found.')
        end
    end
    
    % Sacrum m2m data scaling
    stepSize = mean(diff((plate.HS)))/plate.Freq*subject.mVel;
    [capture.m2mSacCor.Z, corScale] = ...
        comCorZ(capture.m2mSac.Z, subject.Height, stepSize, viz);
    capture.sacCorScale = corScale;
    
    % Reference CoM and Minimum to minimum CoM from GRF
    %     [~, e1] = postZero(plate.F1.Z(analRange));
    %     [~, e2] = postZero(plate.F2.Z(analRange));
    %     fSum = plate.F1.Z(analRange)+plate.F2.Z(analRange)-e1-e2;
    zAcc = (plate.F1.Z+plate.F2.Z)/subject.Mass-g;
    [plate.hhTime, plate.stepCoM.Z, plate.stepVel.Z] = ...
        acc2com(plate.Time,zAcc,plate.HS,plate.TO,analStep,viz);
    [plate.m2mTime, plate.m2mCoM.Z, plate.m2mVel.Z] = ...
        acc2mm(plate.Time,zAcc,plate.HS,plate.TO,analStep,viz);
    
   % draw plot
    if viz
        figure, hold on, grid on
        plot(capture.Time, capture.RHip.Z,'-o',...
            'MarkerIndices', HS, 'MarkerEdgeColor', 'red', 'MarkerFaceColor', 'red', 'MarkerSize', 3)
        plot(capture.Time(TO), capture.RHip.Z(TO),'o',...
            'MarkerEdgeColor', 'blue', 'MarkerFaceColor', 'blue', 'MarkerSize', 3)
        title('Right Hip in Z with HS index')
        xlabel('time (sec)'), ylabel('Z distance (mm)')
        xlim([capture.Time(HS(1)) capture.Time(HS(end))])
        hold off
        
        figure, hold on
        rlColor = 'bk';
        for i = 1:subject.AnalStep
            plot(plate.stepTime{i}, [plate.stepGRF(i).Y, plate.stepGRF(i).Z] ,'Color',rlColor(mod(i,2)+1))
        end
        title([num2str(subject.AnalStep) ' steps GRF'])
        xlabel('time (sec)'), ylabel('Force (N)')
        hold off
        
        figure
        subplot(2,1,1), hold on
        checkRange = 3000:3500;
        Rheel = capture.LHeel.Z(checkRange)-min(capture.LHeel.Z);
        Rpla = plate.F1.Z(checkRange)-min(plate.F1.Z);
        plot(capture.Time(checkRange),Rheel/max(Rheel))
        plot(plate.Time(checkRange),Rpla/max(Rpla))
        axis tight
        title('Sync Check (Left)')
        legend('Left Heel Z (after)', 'Left Force Plate Z (after)')
        subplot(2,1,2), hold on
        checkRange = 1:1000;
        Rheel = capture.LHeel.Z(checkRange)-min(capture.LHeel.Z);
        Rpla = plate.F1.Z(checkRange)-min(plate.F1.Z);
        plot(capture.Time(checkRange),Rheel/max(Rheel))
        plot(plate.Time(checkRange),Rpla/max(Rpla))
        axis tight
        title('Jump Synal (Left)'), xlabel('time (sec)')

        %         alignFigures
    end
        
    if modeSave == true
        save([fileName '.mat'],'subject', 'capture', 'plate')
        fprintf('Data %s.mat is saved!\n', fileName)
    end
end