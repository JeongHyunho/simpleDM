close all
subj = ['KYS';'KCM';'LJY';'CCH';'KJH';'KHS';'ISH'];
speed = ['L';'M';'F'];
for subi = 7:length(subj)
    cd(fileparts(mfilename('fullpath')))
    Subject_name = subj(subi,:);

    cd([pwd '\' Subject_name])
    for spi = 3:3
        Walking_speed = speed(spi);
        Trials = {'1';'2';'3'};
        save_mode = true;
        viz = true;

        for j = 1:length(Trials)
            file_name = [Subject_name '_' Walking_speed '_' Trials{j}];
            WalkingDataEncode(file_name, [10 30], save_mode, viz);
%             pause
            close all
        end
    end
end


function [subject, capture, plate, cam] = WalkingDataEncode(filename, cut_off_freq, mode_save, viz)

    switch nargin
        case 1
            cut_off_freq = [10 15];
            mode_save = true;
            viz = false;
        case 2
            mode_save = true;
            viz = false;
        case 3
            viz = false;
    end
    
    Info = strsplit(filename,'_');
    subject.Name = Info{1};
    subject.Walking_speed = Info{2};
    subject.Trial = Info{3};

    load([fileparts(mfilename('fullpath')) '\' 'SubInfo'])
    sub_lgc = strcmp(subject.Name,SubInfo.Name);
    if ~any(sub_lgc), error('No name in SubInfo.'); end
    height = SubInfo.Height(sub_lgc)*0.01;
    subject.Height = height;
    walk_vel = SubInfo.(subject.Walking_speed)(sub_lgc);
    
    capture = struct('step_time',[],'step_CoM',[],'freq',[],'cut_off_freq',[]);
    [capture_raw, capture_txt] = xlsread([filename '.trc.xls']);

    marker_num = (size(capture_txt, 2) - 2)/3;
    capture.time = capture_raw(7:end, 2);
    capture.freq = 1/mean(capture.time(2:end) - capture.time(1:end-1));
    [b_capture, a_capture] = butter(5, cut_off_freq(1)/capture.freq);
    capture.cut_off_freq = cut_off_freq(1);

    for i = 1:marker_num
        marker_name = capture_txt{4,3*i};
        capture.(marker_name).X = filtfilt(b_capture,a_capture, capture_raw(7:end, 3*i))/1000;
        capture.(marker_name).Y = filtfilt(b_capture,a_capture, capture_raw(7:end, 3*i+1))/1000;
        capture.(marker_name).Z = filtfilt(b_capture,a_capture, capture_raw(7:end, 3*i+2))/1000;
    end
        
    plate = struct('step_time',[],'step_GRF',[],'freq',[],'cut_off_freq',[]);
    [force_raw, force_txt] = xlsread([filename '.anc.xls']);
    g = 9.81;

    time_raw = force_raw(12:end,1 );
    plate_freq = 1/mean(time_raw(2:end) - time_raw(1:end-1));
    r = round(plate_freq/capture.freq);
    
    for i = 1:13
        force_deci(:,i) = decimate(force_raw(12:end, i),r);
    end
    
    plate.time =capture.time;
    plate.freq = capture.freq;
    [b_force, a_force] = butter(5, cut_off_freq(2)/plate.freq);
    plate.cut_off_freq = cut_off_freq(2);

    sens = [0.5 -0.5 1 0.6 0.3 0.3 0.5 -0.5 1 0.6 0.3 0.3]*5000/2^15;
    for i = 1:12
        force_name = force_txt{9, 1+i};
        plate.(force_name(1:2)).(force_name(3)) = filtfilt(b_force, a_force,force_deci(:, i+1)*sens(i));
    end
    subject.M = mean(plate.F1.Z(1:round(end/10))+plate.F2.Z(1:round(end/10)))/g;
    Racc = deriv5(deriv5(capture.R_heel.Z(1:10*capture.freq),0.01)',0.01)';
    Lacc = deriv5(deriv5(capture.L_heel.Z(1:10*capture.freq),0.01)',0.01)';
    subplot(3,1,1), plot([Racc+Lacc ((plate.F1.Z(1:10*plate.freq)+plate.F2.Z(1:10*plate.freq))-subject.M*g)./subject.M])
    [Mpks,Mlocs] = findpeaks(Racc+Lacc);
    [Fpks,Flocs] = findpeaks(((plate.F1.Z(1:10*plate.freq)+plate.F2.Z(1:10*plate.freq))-subject.M*g)./subject.M);
    index = [];
    for i = 1:length(Mpks)
        if Mpks(i) < max(Mpks)*0.6
            index = [index;i];
        end
    end
    Mpks(index) = [];
    Mlocs(index) = [];
    index = [];
    for i = 1:length(Fpks)
        if Fpks(i) < max(Fpks)*0.6
            index = [index;i];
        end
    end
    Fpks(index) = [];
    Flocs(index) = [];
%     subplot(2,1,1), plot(Mlocs,Mpks,'o',Flocs,Fpks,'*')
    assert(length(Mpks) == length(Fpks),['M peak is ',num2str(length(Mpks)), ' and F peak is ', num2str(length(Fpks))])
    
    delay = round(mean(Flocs-Mlocs));
    
    fprintf('\n Delay Time : %.2d \n',delay/capture.freq)

    if delay >= 0
        cap_range = 1:numel(capture.time)-delay;
        pla_range = 1+delay:numel(plate.time);
    else
        cap_range = 1-delay:numel(capture.time);
        pla_range = 1:numel(plate.time)+delay;
    end
    
    capture.time = capture.time(cap_range)-capture.time(cap_range(1));
    plate.time = plate.time(pla_range)-plate.time(pla_range(1));
    
    cap_field = fieldnames(capture);
    pla_field = fieldnames(plate);
    
    for i = 6:numel(cap_field)
        capture.(cap_field{i}).X = capture.(cap_field{i}).X(cap_range);
        capture.(cap_field{i}).Y = capture.(cap_field{i}).Y(cap_range);
        capture.(cap_field{i}).Z = capture.(cap_field{i}).Z(cap_range);
    end
    
    for i = 6:numel(pla_field)
        plate.(pla_field{i}).X = plate.(pla_field{i}).X(pla_range);
        plate.(pla_field{i}).Y = plate.(pla_field{i}).Y(pla_range);
        plate.(pla_field{i}).Z = plate.(pla_field{i}).Z(pla_range);
    end    
    
    Racc = deriv5(deriv5(capture.R_sacral.Z(1:10*capture.freq),0.01)',0.01)';
    Lacc = deriv5(deriv5(capture.L_sacral.Z(1:10*capture.freq),0.01)',0.01)';
    [Mpks,Mlocs] = findpeaks(Racc+Lacc);
    [Fpks,Flocs] = findpeaks(((plate.F1.Z(1:10*plate.freq)+plate.F2.Z(1:10*plate.freq))-subject.M*g)./subject.M);
    subplot(3,1,2), plot([Racc+Lacc ((plate.F1.Z(1:10*plate.freq)+plate.F2.Z(1:10*plate.freq))-subject.M*g)./subject.M])
    index = [];
    for i = 1:length(Mpks)
        if Mpks(i) < max(Mpks)*0.5
            index = [index;i];
        end
    end
    Mpks(index) = [];
    Mlocs(index) = [];
    index = [];
    for i = 1:length(Fpks)
        if Fpks(i) < max(Fpks)*0.5
            index = [index;i];
        end
    end
    Fpks(index) = [];
    Flocs(index) = [];
%     subplot(2,1,2), plot(Mlocs,Mpks,'o',Flocs,Fpks,'*'), hold on
%     text(Mlocs(1),Mpks(1)-5,num2str(Mlocs(1)-Flocs(1))),text(Mlocs(2),Mpks(2)-5,num2str(Mlocs(2)-Flocs(2))), hold off 
    
    LHS = []; RHS = []; LTO = []; RTO = [];
    latency = 10;
    for k = latency*round(plate.freq):length(plate.time)-1
        if plate.F1.Z(k) < 40 && plate.F1.Z(k+1) > 40
            LHS = [LHS; k];
        elseif plate.F2.Z(k) < 40 && plate.F2.Z(k+1) > 40
            RHS = [RHS; k];
        elseif plate.F1.Z(k) > 40 && plate.F1.Z(k+1) < 40
            LTO = [LTO; k];
        elseif plate.F2.Z(k) > 40 && plate.F2.Z(k+1) < 40
            RTO = [RTO; k];
        end
    end
    
    start = [15, 20, 25, 30, 35, 40, 45, 50];
    start_LHS  = start(6);
    
    fprintf('\n %s Start LHS : %d \n\n', filename, start_LHS)
    anal_step = 10;
    subject.anal_step = anal_step;  % 짝수
    subject.start_LHS = start_LHS;
    
    % 왼발 15번째 HS부터 10걸음 분석
    %  --> 1step : LHS - RTO - RHS - LTO - LHS - ... - LHS -RTO-RHS
    LHS = LHS(start_LHS:start_LHS+anal_step/2+1);
    LTO(LTO < LHS(1)) = []; LTO = LTO(1:anal_step/2+1);
    RHS(RHS < LHS(1)) = []; RHS = RHS(1:anal_step/2+1);
    RTO(RTO < LHS(1)) = []; RTO = RTO(1:anal_step/2+1);
    ref_range = 1:500;
    L0 = mean(atan((capture.L_heel.Z(ref_range)-capture.L_toe.Z(ref_range))./(capture.L_toe.Y(ref_range)-capture.L_heel.Y(ref_range))));
    L_foot = atan2(capture.L_heel.Z-capture.L_toe.Z,capture.L_toe.Y-capture.L_heel.Y)-L0;
    R0 = mean(atan((capture.R_heel.Z(ref_range)-capture.R_toe.Z(ref_range))./(capture.R_toe.Y(ref_range)-capture.R_heel.Y(ref_range))));
    R_foot = atan2(capture.R_heel.Z-capture.R_toe.Z,capture.R_toe.Y-capture.R_heel.Y)-R0;
    
    plot([plate.F2.Z(LHS(1):LHS(2))/1000 R_foot(LHS(1):LHS(2))])
    delay = (RHS(1)-LHS(1))-find(R_foot(LHS(1):LHS(2)) == min(R_foot(LHS(1):LHS(2))));
    fprintf('\n Delay Time : %.2d \n',delay/capture.freq)

    if delay >= 0
        cap_range = 1:numel(capture.time)-delay;
        pla_range = 1+delay:numel(plate.time);
    else
        cap_range = 1-delay:numel(capture.time);
        pla_range = 1:numel(plate.time)+delay;
    end
    
    capture.time = capture.time(cap_range)-capture.time(cap_range(1));
    plate.time = plate.time(pla_range)-plate.time(pla_range(1));
    
    cap_field = fieldnames(capture);
    pla_field = fieldnames(plate);
    
    for i = 6:numel(cap_field)
        capture.(cap_field{i}).X = capture.(cap_field{i}).X(cap_range);
        capture.(cap_field{i}).Y = capture.(cap_field{i}).Y(cap_range);
        capture.(cap_field{i}).Z = capture.(cap_field{i}).Z(cap_range);
    end
    
    for i = 6:numel(pla_field)
        plate.(pla_field{i}).X = plate.(pla_field{i}).X(pla_range);
        plate.(pla_field{i}).Y = plate.(pla_field{i}).Y(pla_range);
        plate.(pla_field{i}).Z = plate.(pla_field{i}).Z(pla_range);
    end    
    
    LHS = []; RHS = []; LTO = []; RTO = [];
    latency = 10;
    for k = latency*round(plate.freq):length(plate.time)-1
        if plate.F1.Z(k) < 40 && plate.F1.Z(k+1) > 40
            LHS = [LHS; k];
        elseif plate.F2.Z(k) < 40 && plate.F2.Z(k+1) > 40
            RHS = [RHS; k];
        elseif plate.F1.Z(k) > 40 && plate.F1.Z(k+1) < 40
            LTO = [LTO; k];
        elseif plate.F2.Z(k) > 40 && plate.F2.Z(k+1) < 40
            RTO = [RTO; k];
        end
    end
    
    start = [15, 20, 25, 30, 35, 40, 45, 50];
    start_LHS  = start(4);
    
    fprintf('\n %s Start LHS : %d \n\n', filename, start_LHS)
    anal_step = 10;
    subject.anal_step = anal_step;  % 짝수
    subject.start_LHS = start_LHS;
    
    % 왼발 15번째 HS부터 10걸음 분석
    %  --> 1step : LHS - RTO - RHS - LTO - LHS - ... - LHS -RTO-RHS
    LHS = LHS(start_LHS:start_LHS+anal_step/2+1);
    LTO(LTO < LHS(1)) = []; LTO = LTO(1:anal_step/2+1);
    RHS(RHS < LHS(1)) = []; RHS = RHS(1:anal_step/2+1);
    RTO(RTO < LHS(1)) = []; RTO = RTO(1:anal_step/2+1);
    
    HS = sort([RHS; LHS]);
    TO = sort([RTO; LTO]);
    
    plate.HS = HS; plate.TO = TO;
    plate.RHS = RHS; plate.LHS = LHS;
    
    L_foot = atan2(capture.L_heel.Z-capture.L_toe.Z,capture.L_toe.Y-capture.L_heel.Y)-L0;
    R_foot = atan2(capture.R_heel.Z-capture.R_toe.Z,capture.R_toe.Y-capture.R_heel.Y)-R0;
    
    plot([plate.F2.Z(LHS(1):LHS(2))/1000 R_foot(LHS(1):LHS(2))])
    % GRF CoM. step_CoM
    
    SacX = (capture.R_sacral.X + capture.L_sacral.X)/2;
    SacY = (capture.R_sacral.Y + capture.L_sacral.Y)/2;
    SacZ = (capture.R_sacral.Z + capture.L_sacral.Z)/2;
    Racc = deriv5(deriv5(capture.R_sacral.Z(1:10*capture.freq),0.01)',0.01)';
    Lacc = deriv5(deriv5(capture.L_sacral.Z(1:10*capture.freq),0.01)',0.01)';
    subplot(3,1,3), plot([Racc+Lacc ((plate.F1.Z(1:10*plate.freq)+plate.F2.Z(1:10*plate.freq))-subject.M*g)./subject.M])
    plate.step_time = cell(anal_step,1);
    capture.step_time = cell(anal_step,1);
    plate.step_GRF = struct('X',cell(anal_step,1),'Y',cell(anal_step,1),'Z',cell(anal_step,1));
    capture.step_CoM =  struct('X',cell(anal_step,1),'Y',cell(anal_step,1),'Z',cell(anal_step,1));
    for i = 1:anal_step
        step_range = HS(i):TO(i+1);
        plate.step_time{i} = (0:TO(i+1)-HS(i))'/capture.freq;
        capture.step_time{i} = (0:TO(i+1)-HS(i))'/capture.freq;
        capture.step_CoM(i).X = SacX(step_range)-SacX(step_range(1));
        capture.step_CoM(i).Y = SacY(step_range)-SacY(step_range(1))+walk_vel*capture.step_time{i};
        capture.step_CoM(i).Z = SacZ(step_range)-SacZ(step_range(1));
        
        grf_range = HS(i):TO(i+1);
        if any(RHS == HS(i))
            plate.step_GRF(i).X = plate.F2.X(grf_range);
            plate.step_GRF(i).Y = plate.F2.Y(grf_range);
            plate.step_GRF(i).Z = plate.F2.Z(grf_range);
        elseif any(LHS == HS(i))
            plate.step_GRF(i).X = plate.F1.X(grf_range);
            plate.step_GRF(i).Y = plate.F1.Y(grf_range);
            plate.step_GRF(i).Z = plate.F1.Z(grf_range);
        else
            error('HS index 찾을 수 없음.')
        end
    end
    
    
   % draw plot
    if viz
        
%         fftPlot(capture_raw(7:end, 3*21+2),capture.freq,'on',5);
%         title('FFT of  Z-axis Left Heel data')
%         ylabel('Power Sectrum |X(f)|^2')
%         xlabel('frequncy (Hz)')
%         axes = gca; axes.YGrid = 'on';
%         line([capture.cut_off_freq, capture.cut_off_freq],[axes.YLim(1) axes.YLim(2)], ...
%             'Color', 'k','LineWidth',2,'LineStyle','-.');
%         legend('Power Spectrum', 'Cut-off Freq')       
%         grid on
%         
%         fftPlot(force_raw(12:end,4),plate.freq*r,'on',6);
%         title('FFT of 2nd plate Z force data')
%         ylabel('Power Sectrum |X(f)|^2')
%         xlabel('frequncy (Hz)')
%         axes = gca; axes.YGrid = 'on';
%         line([plate.cut_off_freq, plate.cut_off_freq],[axes.YLim(1) axes.YLim(2)], ...
%             'Color', 'k','LineWidth',2,'LineStyle','-.');
%         legend('Power Spectrum', 'Cut-off Freq')
%         grid on

%         figure(10), hold on, grid on
%         plot(capture.time, capture.R_hip.Z,'-o',...
%             'MarkerIndices', HS, 'MarkerEdgeColor', 'red', 'MarkerFaceColor', 'red', 'MarkerSize', 3)
%         title('R hip.Z with HS index')
%         xlabel('time (sec)'), ylabel('Z distance (mm)')
%         xlim([capture.time(HS(1)) capture.time(HS(end))])
%         hold off
        
        figure(13), hold on
        rl_color = 'bk';
        for i = 1:subject.anal_step
            plot(plate.step_time{i}, [plate.step_GRF(i).Y, plate.step_GRF(i).Z] ,'Color',rl_color(mod(i,2)+1))
        end
        title([num2str(subject.anal_step) ' steps GRF'])
        xlabel('time (sec)'), ylabel('Force (N)')
        hold off

        figure(14)
        for i = 1:subject.anal_step
            subplot(1,3,1), hold on
            plot(capture.step_time{i}, capture.step_CoM(i).X,'Color','k')            
            subplot(1,3,2), hold on
            plot(capture.step_time{i}, capture.step_CoM(i).Y,'Color','k')
            subplot(1,3,3), hold on
            plot(capture.step_time{i}, capture.step_CoM(i).Z,'Color','k')
        end        
        subplot(1,3,1)
        ylabel('CoM (m)')
        subplot(1,3,2)
        title('step CoM, X-Y-Z')
        xlabel('time (sec)')
        hold off
        
        figure(16)
        subplot(2,1,1), hold on
        check_range = 1000:3500;
        ref_range = 1:500;
%         L0 = mean(atan((capture.L_heel.Z(ref_range)-capture.L_toe.Z(ref_range))./(capture.L_toe.Y(ref_range)-capture.L_heel.Y(ref_range))));
        L_foot = atan2(capture.L_heel.Z-capture.L_toe.Z,capture.L_toe.Y-capture.L_heel.Y)-L0;
%         R0 = mean(atan((capture.R_heel.Z(ref_range)-capture.R_toe.Z(ref_range))./(capture.R_toe.Y(ref_range)-capture.R_heel.Y(ref_range))));
        R_foot = atan2(capture.R_heel.Z-capture.R_toe.Z,capture.R_toe.Y-capture.R_heel.Y)-R0;
        Rpla = plate.F1.Z(check_range)-min(plate.F1.Z);
        plot(capture.time(check_range),L_foot(check_range)/max(L_foot))
        plot(plate.time(check_range),Rpla/max(Rpla))
        axis tight
        title('Sync Check (Left)'), xlabel('time (sec)')
        legend('Left Heel Z (after)', 'Left Force Plate Z (after)')
        subplot(2,1,2),plot(L_foot(LHS(1):LTO(1)))
        
    end
        
    if mode_save == true
        save([filename '.mat'],'subject', 'capture', 'plate')
        fprintf('Data %s.mat is saved!\n', filename)
    end

end

function [fx, P1, handle] = fftPlot(Y,freq,flag,fig)
    X = fft(Y);
    L = length(Y);
    T = L/freq;
    P2 = abs(X)/T;
    P1 = P2(1:floor(L/2)+1);
    P1(2:end-1) = 2*P1(2:end-1);
    fx = freq*(0:floor(L/2))/L;
    if strcmp(flag,  'on')
        figure(fig), handle = semilogy(fx,P1.^2,'k');
    else
        handle = [];
    end
end

function    Idx = CapPlateSync(cap_data, cap_freq, pla_data, pla_freq, latency,viz)
    persistent subp
    
    if isempty(subp)
        subp = 1;
    elseif subp == 1
        subp = 2;
    end
    
    cap_data = cap_data(1:latency*cap_freq);
    pla_data = pla_data(1:latency*pla_freq);
    [~,cap_acc] = deriv5(cap_data, 1/cap_freq);
    
    thresh = 0.70;
    
    cap_acc(cap_acc < thresh*max(cap_acc)) = 0;
    pla_data(pla_data < thresh*max(pla_data)) = 0;
    
    [~, cap_idx] = findpeaks(cap_acc);
    [~, pla_idx] = findpeaks(pla_data);
    
    try
        Idx = mean(pla_idx(1:2)' - cap_idx(1:2));
    catch
        [~, max_cap] = max(cap_acc);
        [~, max_pla] = max(pla_data);
        assert(~isempty(max_cap) || ~isempty(max_pla),' something is wrong.')
        Idx = max_pla-max_cap;
    end
    
    if std(cap_idx) > cap_freq*latency/3 || std(pla_idx) > cap_freq*latency/3
        [~, max_cap] = max(cap_acc);
        [~, max_pla] = max(pla_data);
        assert(~isempty(max_cap) || ~isempty(max_pla),' something is wrong.')
        Idx = max_pla-max_cap;
    end    
    
    if viz
        figure(16)
        subplot(2,2,2+subp), hold on
        plot((0:length(cap_acc)-1)/cap_freq, cap_acc/max(cap_acc))
        plot((0:length(pla_data)-1)/pla_freq, pla_data/max(pla_data))
        legend('Capture Acc (before)', 'Force Plate (before)')
    end
end
