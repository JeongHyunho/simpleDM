close all
% clear all
% subj =  ['KYS';'KCM';'ISH';'KHS';'LJY';'CCH';'KJH'];
speed = 1:4;
time_length=[];
for subi =20
g=9.81;
%     cd(fileparts(mfilename('fullpath')))
    load('SubInfo');
    Subject_name=SubInfo.subjN(subi);

    save_mode = 1;
    save_mode_sync=1;
    save_mode_inverse=0;
    viz = 1;
    target_freq=200;
    cut_off_freq=[10 30;
        20 30];
    for spi =0:4
        
        if spi>=1
            Walking_speed = speed(spi);
            if subi>4
%             time_length=20.5;
            else
%                 time_length=140;
            end
        else
%             time_length=10;
        end
        if subi<=9
            file_name = ['S00' num2str(subi) '_T00' num2str(spi)];
        else
            file_name = ['S0' num2str(subi) '_T00' num2str(spi)];
        end
        
        ratio=0.7;
        % plate / capture
        [subject,capture,capture30,plate,plate30,file_name_save]=WalkingDataEncode(file_name,target_freq,cut_off_freq, save_mode, viz,time_length,ratio);
        if isempty(subject)
            continue
        end

%        % joint
%        
%         tic
            kinematic_calculated=fullBody3Dmodel(file_name_save,viz,viz_angle);
%         toc
%         
% %         tic
% %         kinematic_calculated2=fullBody3Dmodel_v1(file_name,viz);
% %         toc
%         
            joint=InverseDynamics3D(file_name_save,kinematic_calculated,viz,save_mode_inverse);

        if spi>=1
        imu_target_freq=0.00675^-1;
%             imu_target_freq=100;
%             
%             % imu
%         
            Mratio=0.7;
            Iratio=0.7;
            [IMU10, IMU30, IMU_raw]=sync_marker_imu(file_name_save,imu_target_freq,Mratio,Iratio,cut_off_freq,save_mode_sync,viz);
            if isempty(IMU10)
                continue
            end
        end
    end
end


function [subject, capture, capture30, plate, plate30,file_name_save] = WalkingDataEncode(filename, target_freq, cut_off_freq, mode_save, viz,time_length,ratio)
g=9.81;
    switch nargin
        case 1
            assert('give targer frequency you want to get data');
        case 2
            cut_off_freq = [10 30];
            mode_save = true;
            viz = false; 
            time_length=[];
        case 3
            mode_save = true;
            viz = false;
            time_length=[];
        case 4
            viz = false;
            time_length=[];
        case 5
            time_length=[];
    end
    
    fprintf('>>>>     Start %s  Encoding !      <<<<<\n ',  filename);
    Mratio=ratio*0.7;
    Fratio=ratio;
    Info = strsplit(filename,'_');
    subject.subjN = Info{1};
    subject.Walking_speed = Info{2};
    
    load([fileparts(mfilename('fullpath')) '\' 'SubInfo'])
    sub_lgc = strcmp(subject.subjN,SubInfo.subjN);
    if ~any(sub_lgc), error('No name in SubInfo.'); end
    height = SubInfo.Height(sub_lgc)*0.01;
    mass = SubInfo.Mass(sub_lgc);
    
    subi=str2double(subject.subjN(2:end));
    spi=str2double(subject.Walking_speed(2:end));
    
    subject.subi=subi;
    subject.spi=spi;
    subject.Height = height;
    subject.Mass = mass;
    subject.Sex = SubInfo.Sex{sub_lgc};
    subject.Offset1=SubInfo.Offset1(sub_lgc);
    subject.Offset2=SubInfo.Offset2(sub_lgc);
    subject.Hand=SubInfo.Hand(sub_lgc);
    subject.Shoulder=SubInfo.Shoulder(sub_lgc);
    

    if spi>=1
        walk_vel = SubInfo.(subject.Walking_speed)(sub_lgc);
        subject.walk_vel=walk_vel;
        filename_data=filename;
    else
        filename_data=[Info{1} '_static1'];
    end
    
    % capture
    capture = struct('step_time',[],'step_CoM',[],'freq',[],'cut_off_freq',[],'marker_names',[]);
    capture30 = struct('step_time',[],'step_CoM',[],'freq',[],'cut_off_freq',[],'marker_names',[]);
    % Read file, and find header lines
    id = fopen([filename_data '.trc']);
%     id = fopen([filename_data '-Dynamic.trc']);
    if id==-1
        warning('No %s.trc file',filename_data);
        subject=[]; capture=[]; capture30=[]; plate=[]; plate30=[];
        return
    end
    fgetl(id);fgetl(id);fgetl(id);
    marker_names = strsplit(fgetl(id), '\t')';
    marker_axis = strsplit(fgetl(id), '\t');
    marker_num = (length(marker_axis)-2)/3;
    marker_names = marker_names(3:marker_num+2);
    
    [capture_raw_raw] = dlmread([filename_data '.trc'],'\t',5,0);
%     [capture_raw] = dlmread([filename_data '-Dynamic.trc'],'\t',5,0);
  	capture_freq_raw=capture_raw_raw(2,2)^-1;

    % decimate
    if capture_freq_raw==target_freq
        capture_raw=capture_raw_raw;
    else
        capture_raw=zeros(ceil(length(capture_raw_raw(:,1))/(capture_freq_raw/target_freq)),length(capture_raw_raw(1,:)));
        for ii=1:length(capture_raw_raw(1,:))
            capture_raw(:,ii)=decimate(capture_raw_raw(:,ii),capture_freq_raw/target_freq);
        end
    end

    capture.freq = target_freq;
    
    if isempty(time_length)
        data_length_capture=length(capture_raw(:,2));
        
    else
        data_length_capture=round(time_length*capture.freq+1);
    end
    
    
    capture30.time = [0:target_freq^-1:(data_length_capture-1)/target_freq]';   
    
    capture.time = [0:target_freq^-1:(data_length_capture-1)/target_freq]';   
    
    
    
    capture30.freq = 1/mean(capture.time(2:data_length_capture) - capture.time(1:data_length_capture-1));
    [b_capture, a_capture] = butter(5, cut_off_freq(1,1)/(capture.freq/2));
    [b2_capture,a2_capture]= butter(5, cut_off_freq(1,2)/(capture.freq/2));
    capture.cut_off_freq = cut_off_freq(1,1);
    capture30.cut_off_freq =cut_off_freq(1,2);
    
    for i = 1:marker_num
        marker_name = strrep(marker_names{i,1},'.','_');
        capture.marker_names{i,1}=marker_name;
        capture30.marker_names{i,1}=marker_name;
        capture.(marker_name).X = filtfilt(b_capture,a_capture, capture_raw(1:data_length_capture, 3*i))/1000;
        capture.(marker_name).Y = filtfilt(b_capture,a_capture, capture_raw(1:data_length_capture, 3*i+1))/1000;
        capture.(marker_name).Z = filtfilt(b_capture,a_capture, capture_raw(1:data_length_capture, 3*i+2))/1000;
        capture30.(marker_name).X = filtfilt(b2_capture,a2_capture, capture_raw(1:data_length_capture, 3*i))/1000;
        capture30.(marker_name).Y = filtfilt(b2_capture,a2_capture, capture_raw(1:data_length_capture, 3*i+1))/1000;
        capture30.(marker_name).Z = filtfilt(b2_capture,a2_capture, capture_raw(1:data_length_capture, 3*i+2))/1000;
        
    end
    
        
    % plate
    plate = struct('step_time',[],'step_GRF',[],'freq',[],'cut_off_freq',[]);
        
    % Read file, and find header lines
    id = fopen([filename_data '.anc']);
    if id==-1
        warning('No %s.anc file',filename_data);
        subject=[]; capture=[]; capture30=[]; plate=[]; plate30=[];
        return
    end
    fgetl(id);fgetl(id);fgetl(id);fgetl(id);fgetl(id);fgetl(id);fgetl(id);fgetl(id);
    force_names = strsplit(fgetl(id), '\t')';
    force_names = force_names(2:end);
    
    fclose('all');
    
    [force_raw_raw] = dlmread([filename_data '.anc'],'\t',11,0);
    g = 9.81;
    
    plate_freq_raw=force_raw_raw(2,1)^-1;
    
    % decimate
    if plate_freq_raw==target_freq
        force_raw=force_raw_raw;
    else
        force_raw=zeros(ceil(length(force_raw_raw(:,1))/(plate_freq_raw/target_freq)),length(force_raw_raw(1,:)));
        for ii=1:length(force_raw_raw(1,:))
            force_raw(:,ii)=decimate(force_raw_raw(:,ii),plate_freq_raw/target_freq);
        end
    end
    
    plate_freq =target_freq;
    
    if isempty(time_length)
        data_length_plate=length(force_raw(:,2));
    else
        data_length_plate=round(time_length*plate_freq+1);
    end
    
%     time_raw = force_raw(1:data_length_plate,1);
    
    r = round(plate_freq/capture.freq);
    force_deci=zeros(ceil(data_length_capture/r),13);
    for i = 1:13
        force_deci(:,i) = decimate(force_raw(1:data_length_capture, i),r);
    end
    
    plate.time =capture.time;
    plate.freq = capture.freq;
    plate30.time =capture.time;
    plate30.freq = capture.freq;
    [b_force, a_force] = butter(5, cut_off_freq(2,1)/(plate.freq/2));
    [b2_force, a2_force] = butter(5, cut_off_freq(2,2)/(plate.freq/2));
    plate.cut_off_freq = cut_off_freq(2,1);
    plate30.cut_off_freq = cut_off_freq(2,2);
    
    

    
    

    
    %%
    [fx, P1, handle] = fftPlot(force_deci(:,10),plate.freq,'off',1);
    
    force_deci(:,2:7)=force_deci(:,2:7)-SubInfo.Offset1{sub_lgc};
    force_deci(:,8:13)=force_deci(:,8:13)-SubInfo.Offset2{sub_lgc};
    sens = [0.5 -0.5 1 0.6 -0.3 0.3 0.5 -0.5 1 0.6 -0.3 0.3]*5000/2^11;  % 2^N : daq-board
    for i = 1:12
        force_name = force_names{i};
        plate.(force_name(1:2)).(force_name(3)) = filtfilt(b_force, a_force,force_deci(:, i+1)*sens(i));
        plate30.(force_name(1:2)).(force_name(3)) = filtfilt(b2_force, a2_force,force_deci(:, i+1)*sens(i));
    end
    
    
    % cop
    center = [-0.3025 0.3025;0 0 ; 0 0];
    d = 0.116+0.009;
    for ii=1:2
        plate.(['C' num2str(ii)]).X=(plate.(['M' num2str(ii)]).Y+ ...
            d*plate.(['F' num2str(ii)]).X)./plate.(['F' num2str(ii)]).Z+center(1,ii);
        plate.(['C' num2str(ii)]).Y=-(-plate.(['M' num2str(ii)]).X- ...
            d*plate.(['F' num2str(ii)]).Y)./plate.(['F' num2str(ii)]).Z+center(2,ii);
        plate.(['C' num2str(ii)]).Z=zeros(length(plate.time),1)+center(3,ii);
        
        plate30.(['C' num2str(ii)]).X=(plate30.(['M' num2str(ii)]).Y+ ...
            d*plate30.(['F' num2str(ii)]).X)./plate30.(['F' num2str(ii)]).Z+center(1,ii);
        plate30.(['C' num2str(ii)]).Y=-(-plate30.(['M' num2str(ii)]).X- ...
            d*plate30.(['F' num2str(ii)]).Y)./plate30.(['F' num2str(ii)]).Z+center(2,ii);
        plate30.(['C' num2str(ii)]).Z=zeros(length(plate30.time),1)+center(3,ii);
    end
    
    
    [plate,plate30,capture,capture30]=sync_marker_plate(subject,Mratio,Fratio,capture,plate,capture30,plate30,viz);
    
    
    
    
    
    
%     try
        % segmentation
        if spi>=1 
            [subject,plate,capture,plate30,capture30]=phase_index(filename_data,subject,plate,capture,plate30,capture30,viz);
            
        end
        
%     catch ME
%         disp( getReport(ME, 'extended', 'hyperlinks', 'on' ) );
%         warning(['can not find HS for' filename]);
%         
%     end
    
    if spi==0
        filename_data=[Info{1} '_T000'];
    end
    if isempty(time_length)
        file_name_save=[filename_data];
        if mode_save
            
    %         save([filename '.mat'],'subject', 'capture', 'plate','capture30','plate30')
            save([file_name_save '.mat'],'subject', 'capture', 'plate','capture30','plate30');
            fprintf('\nData %s.mat is saved!\n', filename);
        end
    else
        file_name_save=[filename_data '_' num2str(time_length) ];
        if mode_save
            
        %         save([filename '.mat'],'subject', 'capture', 'plate','capture30','plate30')
            save([file_name_save '.mat'],'subject', 'capture', 'plate','capture30','plate30');
            fprintf('\nData %s.mat is saved!\n', filename);
        end
    end
end

function [plate,plate30,capture,capture30]=sync_marker_plate(subject,Mratio,Fratio,capture,plate,capture30,plate30,viz_sync)
switch nargin
    case 7 
        viz_sync=0;
end
g=9.81;

    subi=str2double(subject.subjN(2:end));
    spi=str2double(subject.Walking_speed(2:end));
    if spi==0
        return
    end
    time_cmp=7;
    [~,Racc] = deriv5(capture.R_Heel.Z(1:time_cmp*capture.freq),0.01);
    [~,Lacc] = deriv5(capture.L_Heel.Z(1:time_cmp*capture.freq),0.01);
    Macc=Racc+Lacc;
    Fzz=((plate.F1.Z(1:time_cmp*plate.freq)+plate.F2.Z(1:time_cmp*plate.freq))-subject.Mass*g)./subject.Mass;
if viz_sync
    figure(subi),
    set(gcf,'Position',[100+subi*2 200 1800 700]);
    subplot(4,8,spi*2-1), plot([Macc Fzz]);
end
    if any(subi*10+spi == [ 53 134])
        Fratio=0.88;
    elseif any(subi*10+spi == [72 173 194 201 203])
        Mratio=0.61;
    elseif any(subi*10+spi == [94])
        Fratio=0.8;
    elseif any(subi*10+spi == [111 121])
        Fratio=0.576;
    elseif any(subi*10+spi == [112])
        Mratio=0.5;
        Fratio=0.72;
    end
    
    
    [Mpks,Mlocs] = findpeaks(Macc,'MinPeakHeight',max(Macc)*Mratio);
    [Fpks,Flocs] = findpeaks(Fzz,'MinPeakHeight',max(Fzz)*Fratio);
    if subi*10+spi == 91
        Flocs=[Flocs;252];
%         Mlocs=Mlocs([1;3]);
    elseif subi*10+spi == 23
        Flocs=Flocs([1; 3]);
    elseif (length(Mpks)~=length(Fpks)) || (length(Mpks) == 1)
        warning(['M peaks are ',num2str(length(Mpks)), ' and Force peaks are ', num2str(length(Fpks))])
        Mratio=Mratio*0.6;
        Fratio=Fratio*0.6;
        [Mpks,Mlocs] = findpeaks(Macc,'MinPeakHeight',max(Macc)*Mratio);
        [Fpks,Flocs] = findpeaks(Fzz,'MinPeakHeight',max(Fzz)*Fratio);
        assert((length(Mpks)==length(Fpks) && length(Mpks) ~= 1),['M peak is ',num2str(length(Mpks)), ' and Force peak is ', num2str(length(Fpks))]);
        fprintf(['M peaks are ',num2str(length(Mpks)), ' and Force peaks are ', num2str(length(Fpks))]);
    end
    
    index_delay = round(mean(Flocs-Mlocs));
    
    fprintf('\n Force Delay Time : %.2d \n',index_delay/capture.freq)
    
   
    if index_delay >= 0
        capture_range = 1:numel(capture.time)-index_delay;
        plate_range = 1+index_delay:numel(plate.time);
    else
        capture_range = 1-index_delay:numel(capture.time);
        plate_range = 1:numel(plate.time)+index_delay;
    end
    
    capture.time = capture.time(capture_range)-capture.time(capture_range(1));
    capture30.time = capture30.time(capture_range)-capture30.time(capture_range(1));
    plate.time = plate.time(plate_range)-plate.time(plate_range(1));
    plate30.time = plate30.time(plate_range)-plate30.time(plate_range(1));

% capture
    AXII='XYZ';
    for i = 1:length(capture.marker_names)
        
        marker_name=capture.marker_names{i,1};
        for axii=1:3
            capture.(marker_name).(AXII(axii))= capture.(marker_name).(AXII(axii))(capture_range);
            capture30.(marker_name).(AXII(axii)) = capture30.(marker_name).(AXII(axii))(capture_range);
        end
    end

% plate
    force_names='FMC';

    for platei=1:2
        for forcei=1:3
            for axii=1:3
                plate.([force_names(forcei) num2str(platei)]).(AXII(axii))= ...
                    plate.([force_names(forcei) num2str(platei)]).(AXII(axii))(plate_range);
                plate30.([force_names(forcei) num2str(platei)]).(AXII(axii))= ...
                    plate30.([force_names(forcei) num2str(platei)]).(AXII(axii))(plate_range);
            end
        end
    end
    [~,Racc] = deriv5(capture.R_Heel.Z(1:time_cmp*capture.freq),0.01);
    [~,Lacc] = deriv5(capture.L_Heel.Z(1:time_cmp*capture.freq),0.01);
    Macc=Racc+Lacc;
    Fzz=((plate.F1.Z(1:time_cmp*plate.freq)+plate.F2.Z(1:time_cmp*plate.freq))-subject.Mass*g)./subject.Mass;
if viz_sync
    figure(subi),

    subplot(4,8,spi*2+7),plot([Macc Fzz]);
    
end
    
	
                    
end


function [IMU_raw,IMU10, IMU30]=sync_marker_imu(filename,target_freq,Mratio,Iratio,cut_off_freq,save_mode_sync,viz)
    switch nargin
        case 1
            assert('give target frequency');
        case 2
            Mratio=0.7;
            Iratio=0.7;
            cut_off_freq=[10 30];
            save_mode_sync=0;
            viz=0;
        case 4
            cut_off_freq=[10 30];
            save_mode_sync=0;
            viz=0;
        case 5
            save_mode_sync=0;
            viz=0;
        case 6
            viz=0;
    end
    
    try
        load([filename '.mat']);
    catch
        IMU_raw=[];IMU10=[];IMU30=[];
        return
    end
    g=9.81;
    % imu
   
    file_name_imu=[subject.subjN '\' subject.subjN '_v' sprintf('%.1f',subject.walk_vel) '.csv'];
    subi=str2double(subject.subjN(2:end));
    spi=str2double(subject.Walking_speed(2:end));
    id = fopen(file_name_imu);
    if id==-1
        warning('No %s.csv IMU data file',file_name_imu);
        IMU_raw=[];IMU10=[];IMU30=[];
        return
    end
    if str2double(subject.subjN(2:end))<=4
        start_line=200;    
    else
        start_line=130;
    end
    
    for ii=1:start_line
    fgetl(id);
    end
    imu_names_raw = strsplit(fgetl(id), ',')';

    fclose('all');
    
    raw_raw_data=dlmread(file_name_imu,',',start_line+1,0);
    end_line=min(find(raw_raw_data(2:end,1)==0));
    if isempty(end_line)
        raw_raw_data=raw_raw_data(1:end,:);
    else
        raw_raw_data=raw_raw_data(1:end_line,:);
    end
    
    % resample
    if raw_raw_data(2,1)^-1==target_freq
        raw_data=raw_raw_data;
        raw_data_time=raw_raw_data(:,1);
    else
        for ii=1:length(raw_raw_data(1,:))
            [raw_data(:,ii),raw_data_time]=resample(raw_raw_data(:,ii),raw_raw_data(:,1),target_freq);
        end
        
        
    end
    
    IMU_raw = struct('step_time',[],'step_acc',[],'freq',[],'cut_off_freq',[]);
    IMU_raw.time=raw_data_time;
    IMU_raw.freq=(IMU_raw.time(2)-IMU_raw.time(1))^-1;
%     IMU_raw.freq=148/0.999;
    
    
    IMU_raw.imu_names=[];
    
    if str2double(subject.subjN(2:end))>4
        for ii=1:length(imu_names_raw)-1
            names=strsplit(imu_names_raw{ii+1,1});
            if length(names{3})==1
                IMU10=[];
                continue
            end
            IMU_raw.imu_names{str2double(names{1}(1)),1}=(names{1}(3:end-1));
            IMU_raw.(names{1}(3:end-1)).(names{2}).unit=(names{5}(2:end-1));
            IMU_raw.(names{1}(3:end-1)).(names{2}).(names{3}(end))=raw_data(:,ii+1);
        end
        
    else
        for ii=1:length(imu_names_raw)/2
            names=strsplit(imu_names_raw{ii*2,1});
            if length(names{3})==1
                continue
            end
            IMU_raw.imu_names{str2double(names{1}(1)),1}=(names{1}(3:end-1));
            IMU_raw.(names{1}(3:end-1)).(names{2}).unit=(names{5}(2:end-1));
            IMU_raw.(names{1}(3:end-1)).(names{2}).(names{3}(end))=raw_data(:,ii*2);
        end
    end
    imu_names=IMU_raw.imu_names;
    
    % axis_change
    imu_axis='XZY';
    plate_axis='XYZ';
    DATAi={'Acc','Gyro','Mag'};
    for ii=1:3
        for datai=1:2
            for axii=1:3
                data.(plate_axis(axii))=-IMU_raw.(imu_names{ii,1}).(DATAi{datai}).(imu_axis(axii));
            end
            for axii=1:3
                IMU_raw.(imu_names{ii,1}).(DATAi{datai}).(plate_axis(axii))= ...
                    data.(plate_axis(axii));
            end
        end
    end
    
    IMU10=IMU_raw;
    IMU30=IMU_raw;

    % imu filtering

    [b_imu, a_imu] = butter(5, cut_off_freq(1,1)/(IMU_raw.freq/2));
    [b2_imu,a2_imu]= butter(5, cut_off_freq(1,2)/(IMU_raw.freq/2));
    IMU10.cut_off_freq = cut_off_freq(1,1);
    IMU30.cut_off_freq = cut_off_freq(1,2);
    
    AXII='XYZ';
    Axii='xyz';
    for ii=1:3
        for datai=1:2
            for axii=1:3
                
                IMU10.(imu_names{ii,1}).(DATAi{datai}).(AXII(axii))= ...
                    filtfilt(b_imu,a_imu, IMU_raw.(imu_names{ii,1}).(DATAi{datai}).(AXII(axii)));
                IMU30.(imu_names{ii,1}).(DATAi{datai}).(AXII(axii))= ...
                    filtfilt(b2_imu,a2_imu, IMU_raw.(imu_names{ii,1}).(DATAi{datai}).(AXII(axii)));
            end
        end
    end

norm_Isac=sqrt(sum([IMU10.sacrum.Acc.X IMU10.sacrum.Acc.Y IMU10.sacrum.Acc.Z].^2,2))*g;
[~,axis_Macc]=deriv5([capture.V_Sacral.X capture.V_Sacral.Y capture.V_Sacral.Z],capture.freq^-1);
Macc=sqrt(sum((axis_Macc(1:round(10*capture.freq),:)+[0 0 g]).^2,2));

Iacc=norm_Isac(1:round(10*IMU10.freq),:);
if viz
figure(subi),
set(gcf,'Position',[100+subi*2 200 1800 700]);
subplot(4,8,spi*2),plot(capture.time(1:10*capture.freq,1),Macc); hold on; plot(IMU10.time(1:round(10*IMU10.freq),:),Iacc);
title([subject.subjN '_' subject.Walking_speed]);
end

if (subi==2 && spi==3) || (subi==9 && spi==2)
    Mratio=0.95;
    Iratio=0.95;
elseif (subi==3 && spi==1) || (subi==5 && spi==1) || (subi==8 && spi==4) ...

    Mratio=0.8;
    Iratio=0.8;
end


[Mpks,Mlocs] = findpeaks(Macc,'MinPeakHeight',max(Macc)*Mratio);
[Ipks,Ilocs] = findpeaks(Iacc,'MinPeakHeight',max(Iacc)*Iratio);

if length(Mpks)~=length(Ipks)
    warning(['M peaks are ',num2str(length(Mpks)), ' and IMU peaks are ', num2str(length(Ipks))])
    Mratio=Mratio*0.8;
    Iratio=Iratio*0.8;
    [Mpks,Mlocs] = findpeaks(Macc,'MinPeakHeight',max(Macc)*Mratio);
    [Ipks,Ilocs] = findpeaks(Iacc,'MinPeakHeight',max(Iacc)*Iratio);
    assert(length(Mpks)==length(Ipks),['M peak is ',num2str(length(Mpks)), ' and IMU peak is ', num2str(length(Ipks))])
end

time_delay = mean(capture.time(Mlocs)-IMU10.time(Ilocs));

fprintf('IMU Delay Time : %.2d \n',time_delay)



if time_delay >= 0
    imu_delay = -[1:round(time_delay*IMU10.freq)]';
    imu_range = 1:numel(IMU10.time);
else
    imu_delay = [];
    imu_range = 1-round(time_delay*IMU10.freq):numel(IMU10.time);
end
IMU_raw.time = [flipud(imu_delay)/IMU_raw.freq;IMU_raw.time];
IMU10.time = [flipud(imu_delay)/IMU10.freq;IMU10.time];
IMU30.time = [flipud(imu_delay)/IMU30.freq;IMU30.time];

IMU_raw.time = IMU_raw.time(imu_range(1):end)-IMU_raw.time(imu_range(1));
IMU10.time = IMU10.time(imu_range(1):end)-IMU10.time(imu_range(1));
IMU30.time = IMU30.time(imu_range(1):end)-IMU30.time(imu_range(1));

DATAi={'Acc','Gyro','Mag'};
AXII='XYZ';
for i = 1:length(IMU30.imu_names)
    for datai=1:2
        for axii=1:3
            IMU_raw.(IMU_raw.imu_names{i}).(DATAi{datai}).(AXII(axii))= ...
                [zeros(length(imu_delay),1); ...
                IMU_raw.(IMU_raw.imu_names{i}).(DATAi{datai}).(AXII(axii))(imu_range)];
            IMU10.(IMU10.imu_names{i}).(DATAi{datai}).(AXII(axii))= ...
                [zeros(length(imu_delay),1); ...
                IMU10.(IMU10.imu_names{i}).(DATAi{datai}).(AXII(axii))(imu_range)];
            IMU30.(IMU30.imu_names{i}).(DATAi{datai}).(AXII(axii))= ...
                [zeros(length(imu_delay),1); ...
                IMU30.(IMU30.imu_names{i}).(DATAi{datai}).(AXII(axii))(imu_range)];
        end
    end
end

norm_Isac=sqrt(sum([IMU10.sacrum.Acc.X IMU10.sacrum.Acc.Y IMU10.sacrum.Acc.Z].^2,2))*g;
[~,axis_Macc]=deriv5([capture.V_Sacral.X capture.V_Sacral.Y capture.V_Sacral.Z],capture.freq^-1);
Macc=sqrt(sum((axis_Macc(round(1*capture.freq):round(5*capture.freq),:)+[0 0 g]).^2,2));

Iacc=norm_Isac(round(1*IMU10.freq):round(5*IMU10.freq),:);
Fzz=(sum([plate.F1.Z plate.F2.Z],2)-subject.Mass*g)/subject.Mass+g;
Fzz30=(sum([plate30.F1.Z plate30.F2.Z],2)-subject.Mass*g)/subject.Mass+g;
if viz
subplot(4,8,spi*2+8),plot(capture.time(round(1*capture.freq):round(5*capture.freq),1),Macc);
hold on; plot(IMU10.time(round(1*IMU10.freq):round(5*IMU10.freq),:),Iacc);
plot(plate.time(round(1*plate.freq):round(5*plate.freq),:), ...
    [Fzz(round(1*plate.freq):round(5*plate.freq),:) Fzz30(round(1*plate.freq):round(5*plate.freq),:)]);
end
if ~exist('joint','var')
    joint=[];
end
if save_mode_sync
%     save([filename '.mat'],'subject', 'capture', 'plate','capture30','plate30','joint','kinematic','kinetic','IMU_raw','IMU10','IMU30')
    save([filename '.mat'],'subject', 'capture', 'plate','capture30','plate30','joint','IMU_raw','IMU10','IMU30')
    fprintf('synced Data %s.mat is saved!\n\n', filename)
end


end


function [subject,plate,capture,plate30,capture30]=phase_index(filename,subject,plate,capture,plate30,capture30,viz)
    switch nargin
        case 6
            viz=0;
    end
%     viz=1;
    
    walk_vel=subject.walk_vel;
    subi=subject.subi;
    spi=subject.spi;
    
    LHS = []; RHS = []; LTO = []; RTO = [];
    LHS2= []; RTO2= [];
        latency=10;
        if subi<=4
            latency=60;
        end
        if (subi==7  && spi==1) || (subi == 8 && spi ==1)
            latency=9;
        elseif (subi==9 && spi==1) || (subi==15 && spi==1) || (subi==5 && spi==1)
            latency=8.5;
        elseif (subi==20 && spi==3)
            latency=9.5;
        end

    thred=25;
    for k = round(latency*plate.freq):length(plate.time)-1
        if plate.F1.Z(k) < thred && plate.F1.Z(k+1) >= thred
            LHS = [LHS; k];
        elseif plate.F2.Z(k) < thred && plate.F2.Z(k+1) >= thred
            RHS = [RHS; k];
        elseif plate.F1.Z(k) >= thred && plate.F1.Z(k+1) < thred
            LTO = [LTO; k];
        elseif plate.F2.Z(k) >= thred && plate.F2.Z(k+1) < thred
            RTO = [RTO; k];
        end
    end
    LTO=LTO(LTO > LHS(1));
    RHS=RHS(RHS > LHS(1));
    RTO=RTO(RTO  > LHS(1));
%     data_time=20;
    data_length=length(plate.time);
    % Set start LHS index for save N strindes
    anal_step = 10;
%     td=0;
%     if (subi==6 && spi==3) || (subi==7 && (spi==1 || spi==4)) || (subi==8 && spi==3)

        td=1;
        if (subi==11 && (spi==1 || spi==4)) || (subi==5 && spi==1)
            td=0;

        end
%     end
if subi<=4
    [start_LHS, ~]  = findStartIdx_rnd({plate.F1.Z(1:data_length,1),plate.F2.Z(1:data_length,1)}, ...
        {LHS(LHS<=data_length),RHS(RHS<=data_length)}, {LTO(LTO<=data_length),RTO(RTO<=data_length)}, anal_step/2+1+td,1e-1); % find similar sequence's start index 
            else
            
    [start_LHS, ~]  = findStartIdx({plate.F1.Z(1:data_length,1),plate.F2.Z(1:data_length,1)}, ...
        {LHS(LHS<=data_length),RHS(RHS<=data_length)}, {LTO(LTO<=data_length),RTO(RTO<=data_length)}, anal_step/2+1+td,1e-1); % find similar sequence's start index 
end
    [start_LHS2] = findStartIdx({plate.F1.Z(1:data_length,1)}, {LHS(LHS<=data_length)}, {LTO(LTO<=data_length)}, anal_step/2+1+td,1e-1); % find similar sequence's start index 

    LHS2=LHS(start_LHS2);
    RTO2=RTO(RTO>LHS2&RTO<data_length);
%     RTO2=RTO2(1:anal_step/2+1);
%     start = [15, 20, 25, 30, 35, 40, 45, 50];
%     start_LHS  = start(6);
    
    fprintf('\n %s Start LHS : %d', filename, start_LHS);
    fprintf('\n %s only force Start LHS : %d  \n', filename, start_LHS2);
    
%     anal_step = 10;
    subject.anal_step = anal_step;  % 짝수
    subject.start_LHS = start_LHS;
    
    % 왼발 15번째 HS부터 50걸음 분석
    %  --> 1step : LHS - RTO - RHS - LTO - LHS - ... - LHS -RTO-RHS
    LHS = LHS(start_LHS:start_LHS+anal_step/2+1);
    LTO(LTO < LHS(1)) = []; LTO = LTO(1:anal_step/2+1);
    RHS(RHS < LHS(1)) = []; RHS = RHS(1:anal_step/2+1);
    RTO(RTO < LHS(1)) = []; RTO = RTO(1:anal_step/2+1);
    
    ref_range=1:500;
    L0 = mean(atan((capture.L_Heel.Z(ref_range)-capture.L_Little.Z(ref_range))./(capture.L_Little.Y(ref_range)-capture.L_Heel.Y(ref_range))));
    L_foot = atan2(capture.L_Heel.Z-capture.L_Little.Z,capture.L_Little.Y-capture.L_Heel.Y)-L0;
    R0 = mean(atan((capture.R_Heel.Z(ref_range)-capture.R_Little.Z(ref_range))./(capture.R_Little.Y(ref_range)-capture.R_Heel.Y(ref_range))));
    R_foot = atan2(capture.R_Heel.Z-capture.R_Little.Z,capture.R_Little.Y-capture.R_Heel.Y)-R0;
    
    indd=1;
    if viz
    figure(subi),

    subplot(4,8,spi*2+7),
    plot([plate.F2.Z(LHS(indd):LHS(indd+1))/1000 R_foot(LHS(indd):LHS(indd+1))])
    end
    index_delay = (RHS(indd)-LHS(indd))-find(R_foot(LHS(indd):LHS(indd+1)) == min(R_foot(LHS(indd):LHS(indd+1))));
 
        fprintf('\n Delay Time : %.2d  by foot  \n',index_delay/capture.freq)


     
    if index_delay >= 0
        capture_range = 1:numel(capture.time)-index_delay;
        plate_range = 1+index_delay:numel(plate.time);
    else
        capture_range = 1-index_delay:numel(capture.time);
        plate_range = 1:numel(plate.time)+index_delay;
    end
    
    capture.time = capture.time(capture_range)-capture.time(capture_range(1));
    capture30.time = capture30.time(capture_range)-capture30.time(capture_range(1));
    plate.time = plate.time(plate_range)-plate.time(plate_range(1));
    plate30.time = plate30.time(plate_range)-plate30.time(plate_range(1));
    
    
% capture
    AXII='XYZ';
    for i = 1:length(capture.marker_names)
        
        marker_name=capture.marker_names{i,1};
        for axii=1:3
            capture.(marker_name).(AXII(axii))= capture.(marker_name).(AXII(axii))(capture_range);
            capture30.(marker_name).(AXII(axii)) = capture30.(marker_name).(AXII(axii))(capture_range);
        end
    end

% plate
    force_names='FMC';

    for platei=1:2
        for forcei=1:3
            for axii=1:3
                plate.([force_names(forcei) num2str(platei)]).(AXII(axii))= ...
                    plate.([force_names(forcei) num2str(platei)]).(AXII(axii))(plate_range);
                plate30.([force_names(forcei) num2str(platei)]).(AXII(axii))= ...
                    plate30.([force_names(forcei) num2str(platei)]).(AXII(axii))(plate_range);
            end
        end
    end
        
        
        
    latency=10;
    if subi<=4
        latency=60;
    end
    if (subi==7  && spi==1) || (subi == 8 && spi ==1)
        latency=9;
    elseif (subi==9 && spi==1) || (subi==15 && spi==1) || (subi==5 && spi==1)
        latency=8.5;
    elseif (subi==20 && spi==3)
            latency=9.5;
    end
    
    LHS = []; RHS = []; LTO = []; RTO = [];
    LHS2= []; RTO2=[];

    thred=25;
    for k = round(latency*plate.freq):length(plate.time)-1
        if plate.F1.Z(k) < thred && plate.F1.Z(k+1) >= thred
            LHS = [LHS; k];
        elseif plate.F2.Z(k) < thred && plate.F2.Z(k+1) >= thred
            RHS = [RHS; k];
        elseif plate.F1.Z(k) >= thred && plate.F1.Z(k+1) < thred
            LTO = [LTO; k];
        elseif plate.F2.Z(k) >= thred && plate.F2.Z(k+1) < thred
            RTO = [RTO; k];
        end
    end
    LTO=LTO(LTO > LHS(1));
    RHS=RHS(RHS > LHS(1));
    RTO=RTO(RTO  > LHS(1));
    LHS2=LHS(start_LHS2);
    RTO2=RTO(RTO>LHS2&RTO<data_length);
    
    subject.anal_step = anal_step;  % 짝수
    subject.start_LHS = start_LHS;
    
    % 왼발 15번째 HS부터 50걸음 분석
    %  --> 1step : LHS - RTO - RHS - LTO - LHS - ... - LHS -RTO-RHS
    LHS = LHS(start_LHS:start_LHS+anal_step/2+1);
    LTO(LTO < LHS(1)) = []; LTO = LTO(1:anal_step/2+1);
    RHS(RHS < LHS(1)) = []; RHS = RHS(1:anal_step/2+1);
    RTO(RTO < LHS(1)) = []; RTO = RTO(1:anal_step/2+1);

    
    
    
    % check sync
    
    check_range=1000:1500;
    L0 = mean(atan((capture.L_Heel.Z(check_range)-capture.L_Little.Z(check_range))./(capture.L_Little.Y(check_range)-capture.L_Heel.Y(check_range))));
    L_foot = atan2(capture.L_Heel.Z-capture.L_Little.Z,capture.L_Little.Y-capture.L_Heel.Y)-L0;
    R0 = mean(atan((capture.R_Heel.Z(check_range)-capture.R_Little.Z(check_range))./(capture.R_Little.Y(check_range)-capture.R_Heel.Y(check_range))));
    R_foot = atan2(capture.R_Heel.Z-capture.R_Little.Z,capture.R_Little.Y-capture.R_Heel.Y)-R0;
    
    indd=1;
    if viz
    figure(subi),

    subplot(4,8,spi*2+7),
    plot([plate.F2.Z(LHS(indd):LHS(indd+1))/1000 R_foot(LHS(indd):LHS(indd+1))])
    end
        
    HS = sort([RHS; LHS]);
    TO = sort([RTO; LTO]);
    
    plate.HS = HS; plate.TO = TO;
    plate.RHS = RHS; plate.LHS = LHS;
    plate.RTO = RTO; plate.LTO = LTO;
    
    
    
    % GRF CoM. step_CoM
%     
%     SacX = (capture.R_sacral.X + capture.L_sacral.X)/2;
%     SacY = (capture.R_sacral.Y + capture.L_sacral.Y)/2;
%     SacZ = (capture.R_sacral.Z + capture.L_sacral.Z)/2;
    
    SacX = capture.V_Sacral.X;
    SacY = capture.V_Sacral.Y;
    SacZ = capture.V_Sacral.Z;
    
%     [~,Racc] = deriv5(capture.R_sacral.Z(1:10*capture.freq),0.01);
%     [~,Lacc] = deriv5(capture.L_sacral.Z(1:10*capture.freq),0.01);
%     [~,Racc30] = deriv5(capture30.R_sacral.Z(1:10*capture.freq),0.01);
%     [~,Lacc30] = deriv5(capture30.L_sacral.Z(1:10*capture.freq),0.01);
%     subplot(4,1,4), plot([Racc+Lacc ((plate.F1.Z(1:10*plate.freq)+plate.F2.Z(1:10*plate.freq))-subject.M*g)./subject.M Racc30+Lacc30])
    plate.step_time = cell(anal_step,1);
    capture.step_time = cell(anal_step,1);
    plate.step_GRF = struct('X',cell(anal_step,1),'Y',cell(anal_step,1),'Z',cell(anal_step,1));
    capture.step_CoM =  struct('X',cell(anal_step,1),'Y',cell(anal_step,1),'Z',cell(anal_step,1));
    for i = 1:anal_step
        step_range = HS(i):TO(i+1);
        plate.step_time{i,1} = (0:TO(i+1)-HS(i))'/capture.freq;
        capture.step_time{i,1} = (0:TO(i+1)-HS(i))'/capture.freq;
%         plate.step_time{i,2} = (HS(i):TO(i+1))'/capture.freq;
%         capture.step_time{i,2} = (HS(i):TO(i+1))'/capture.freq;
        
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
    
    grf10_range=LHS(1):RTO(end);
    cap10_range=LHS(1):RTO(end);
    plate.step10_time=(0:RTO(end)-LHS(1))'/capture.freq;
    capture.step10_time=(0:RTO(end)-LHS(1))'/capture.freq;
    plate.GRF10.X=[plate.F1.X(grf10_range) plate.F2.X(grf10_range)];
    plate.GRF10.Y=[plate.F1.Y(grf10_range) plate.F2.Y(grf10_range)];
    plate.GRF10.Z=[plate.F1.Z(grf10_range) plate.F2.Z(grf10_range)];
    capture.CoM10.X=SacX(cap10_range);
    capture.CoM10.Y=SacY(cap10_range)+walk_vel*capture.step10_time;
    capture.CoM10.Z=SacZ(cap10_range);
    
    data_length=length(plate.time);
    data_range=[1:data_length]';
    if viz
        figure(subi),
        set(gcf,'Position',[100+subi*5 200 1500 700]);
        subplot(4,4,8+spi),plot(plate.time(data_range),[plate.F1.Z(data_range) plate.F2.Z(data_range)]);
        hold on; plot(plate.time(grf10_range),[plate.F1.Z(grf10_range) plate.F2.Z(grf10_range)],'k');
        subplot(4,4,12+spi),plot(capture.time(data_range),SacY(data_range)); hold on;
        plot(capture.time(LHS2(1):RTO2(end)),SacY(LHS2(1):RTO2(end)),'color',[0.7 0.7 0.7],'LineWidth',2);
        plot(capture.time(grf10_range),SacY(grf10_range),'k');
        text(5, mean([min(SacY(data_range)) max(SacY(data_range))]), [num2str(start_LHS) '  ' num2str(start_LHS2)]);
        
    end
    
    
   % draw plot
    if 0
        
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
        Rpla = plate.F1.Z(check_range);
        plot(capture.time(check_range),L_foot(check_range)/max(L_foot))
        plot(plate.time(check_range),Rpla/max(Rpla))
        axis tight
        title('Sync Check (Left)'), xlabel('time (sec)')
        legend('Left Heel Z (after)', 'Left Force Plate Z (after)')
        subplot(2,1,2),plot(L_foot(LHS(1):LTO(1)))
       
        
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
function [subject, capture, capture30, plate, plate30,file_name_save] = WalkingDataEncodeJoint(filename, target_freq, cut_off_freq, mode_save, viz,time_length)
g=9.81;
    switch nargin
        case 1
            assert('give targer frequency you want to get data');
        case 2
            cut_off_freq = [10 30];
            mode_save = true;
            viz = false; 
            time_length=[];
        case 3
            mode_save = true;
            viz = false;
            time_length=[];
        case 4
            viz = false;
            time_length=[];
        case 5
            time_length=[];
    end
    
    Info = strsplit(filename,'_');
    subject.subjN = Info{1};
    subject.Walking_speed = Info{2};
    
    load([fileparts(mfilename('fullpath')) '\' 'SubInfo'])
    sub_lgc = strcmp(subject.subjN,SubInfo.subjN);
    if ~any(sub_lgc), error('No name in SubInfo.'); end
    height = SubInfo.Height(sub_lgc)*0.01;
    mass = SubInfo.Mass(sub_lgc);
    
    subi=str2double(subject.subjN(2:end));
    spi=str2double(subject.Walking_speed(2:end));
    
    subject.subi=subi;
    subject.spi=spi;
    subject.Height = height;
    subject.Mass = mass;
    subject.Sex = SubInfo.Sex{sub_lgc};
    subject.Offset1=SubInfo.Offset1(sub_lgc);
    subject.Offset2=SubInfo.Offset2(sub_lgc);
    subject.Hand=SubInfo.Hand(sub_lgc);
    subject.Shoulder=SubInfo.Shoulder(sub_lgc);
    

    if spi>=1
        walk_vel = SubInfo.(subject.Walking_speed)(sub_lgc);
        subject.walk_vel=walk_vel;
        filename_data=filename;
    else
        filename_data=[Info{1} '_static1'];
    end
    
    % capture
    capture = struct('step_time',[],'step_CoM',[],'freq',[],'cut_off_freq',[],'marker_names',[]);
    capture30 = struct('step_time',[],'step_CoM',[],'freq',[],'cut_off_freq',[],'marker_names',[]);
    % Read file, and find header lines
    id = fopen([filename_data '.trc']);
%     id = fopen([filename_data '-Dynamic.trc']);
    if id==-1
        warning('No %s.trc file',filename_data);
        subject=[]; capture=[]; capture30=[]; plate=[]; plate30=[];
        return
    end
    fgetl(id);fgetl(id);fgetl(id);
    marker_names = strsplit(fgetl(id), '\t')';
    marker_axis = strsplit(fgetl(id), '\t');
    marker_num = (length(marker_axis)-2)/3;
    marker_names = marker_names(3:marker_num+2);
    
    [capture_raw_raw] = dlmread([filename_data '.trc'],'\t',5,0);
%     [capture_raw] = dlmread([filename_data '-Dynamic.trc'],'\t',5,0);
  	capture_freq_raw=capture_raw_raw(2,2)^-1;

    % decimate
    if capture_freq_raw==target_freq
        capture_raw=capture_raw_raw;
    else
        capture_raw=zeros(ceil(length(capture_raw_raw(:,1))/(capture_freq_raw/target_freq)),length(capture_raw_raw(1,:)));
        for ii=1:length(capture_raw_raw(1,:))
            capture_raw(:,ii)=decimate(capture_raw_raw(:,ii),capture_freq_raw/target_freq);
        end
    end

    capture.freq = target_freq;
    
    if isempty(time_length)
        data_length_capture=length(capture_raw(:,2));
        
    else
        data_length_capture=round(time_length*capture.freq+1);
    end
    
    
    capture30.time = [0:target_freq^-1:(data_length_capture-1)/target_freq]';   
    
    capture.time = [0:target_freq^-1:(data_length_capture-1)/target_freq]';   
    
    
    
    capture30.freq = 1/mean(capture.time(2:data_length_capture) - capture.time(1:data_length_capture-1));
    [b_capture, a_capture] = butter(5, cut_off_freq(1,1)/(capture.freq/2));
    [b2_capture,a2_capture]= butter(5, cut_off_freq(1,2)/(capture.freq/2));
    capture.cut_off_freq = cut_off_freq(1,1);
    capture30.cut_off_freq =cut_off_freq(1,2);
    
    for i = 1:marker_num
        marker_name = strrep(marker_names{i,1},'.','_');
        capture.marker_names{i,1}=marker_name;
        capture30.marker_names{i,1}=marker_name;
        capture.(marker_name).X = filtfilt(b_capture,a_capture, capture_raw(1:data_length_capture, 3*i))/1000;
        capture.(marker_name).Y = filtfilt(b_capture,a_capture, capture_raw(1:data_length_capture, 3*i+1))/1000;
        capture.(marker_name).Z = filtfilt(b_capture,a_capture, capture_raw(1:data_length_capture, 3*i+2))/1000;
        capture30.(marker_name).X = filtfilt(b2_capture,a2_capture, capture_raw(1:data_length_capture, 3*i))/1000;
        capture30.(marker_name).Y = filtfilt(b2_capture,a2_capture, capture_raw(1:data_length_capture, 3*i+1))/1000;
        capture30.(marker_name).Z = filtfilt(b2_capture,a2_capture, capture_raw(1:data_length_capture, 3*i+2))/1000;
        
    end
    
        
    % plate
    plate = struct('step_time',[],'step_GRF',[],'freq',[],'cut_off_freq',[]);
        
    % Read file, and find header lines
    id = fopen([filename_data '.anc']);
    if id==-1
        warning('No %s.anc file',filename_data);
        subject=[]; capture=[]; capture30=[]; plate=[]; plate30=[];
        return
    end
    fgetl(id);fgetl(id);fgetl(id);fgetl(id);fgetl(id);fgetl(id);fgetl(id);fgetl(id);
    force_names = strsplit(fgetl(id), '\t')';
    force_names = force_names(2:end);
    
    fclose('all');
    
    [force_raw_raw] = dlmread([filename_data '.anc'],'\t',11,0);
    g = 9.81;
    
    plate_freq_raw=force_raw_raw(2,1)^-1;
    
    % decimate
    if plate_freq_raw==target_freq
        force_raw=force_raw_raw;
    else
        force_raw=zeros(ceil(length(force_raw_raw(:,1))/(plate_freq_raw/target_freq)),length(force_raw_raw(1,:)));
        for ii=1:length(force_raw_raw(1,:))
            force_raw(:,ii)=decimate(force_raw_raw(:,ii),plate_freq_raw/target_freq);
        end
    end
    
    plate_freq =target_freq;
    
    if isempty(time_length)
        data_length_plate=length(force_raw(:,2));
    else
        data_length_plate=round(time_length*plate_freq+1);
    end
    
%     time_raw = force_raw(1:data_length_plate,1);
    
    r = round(plate_freq/capture.freq);
    force_deci=zeros(ceil(data_length_capture/r),13);
    for i = 1:13
        force_deci(:,i) = decimate(force_raw(1:data_length_capture, i),r);
    end
    
    plate.time =capture.time;
    plate.freq = capture.freq;
    plate30.time =capture.time;
    plate30.freq = capture.freq;
    [b_force, a_force] = butter(5, cut_off_freq(2,1)/(plate.freq/2));
    [b2_force, a2_force] = butter(5, cut_off_freq(2,2)/(plate.freq/2));
    plate.cut_off_freq = cut_off_freq(2,1);
    plate30.cut_off_freq = cut_off_freq(2,2);
    
    

    
    
    % joint data
    joint = struct('step_time',[],'freq',[],'cut_off_freq',[],'joint_names',[]);
    
    if ~strcmpi(subject.Walking_speed,'static1')
        filename_data=filename;
    else
        filename_data=[strrep(filename,'static1','T000')];
    end
    id = fopen([filename_data '.data']);
    if id==-1
        warning('No %s.data file',filename_data);
        subject=[];
        return
    end
    fgetl(id);fgetl(id);
    joint_names_raw = strsplit(fgetl(id), '\t')';
    joint_names_raw = joint_names_raw(2:end);
    fclose('all');
    [joint_raw_raw] = dlmread([filename_data '.data'],'\t',[4 0 length(capture_raw_raw(:,1))+3 length(joint_names_raw)]);
%     joint_freq_raw=joint_raw_raw(2,1)^-1;
    % decimate
    if capture_freq_raw==target_freq
        joint_raw=joint_raw_raw;
    else
        joint_raw=zeros(ceil(length(joint_raw_raw(:,1))/(capture_freq_raw/target_freq)),length(joint_raw_raw(1,:)));
        for ii=1:length(joint_raw_raw(1,:))
            joint_raw(:,ii)=decimate(joint_raw_raw(:,ii),capture_freq_raw/target_freq);
        end
    end
    
    for i = 1:length(joint_names_raw)
        joint_name = strsplit(joint_names_raw{i,1},' ');
        if length(joint_name)<=2
            continue
        end
        joint_names{i,1}=[joint_name{1} '_' joint_name{2}];
        if length(joint_name)==3
            joint.(joint_names{i,1}).Angle.(joint_name{3})=joint_raw(1:data_length_capture,1+i);
        elseif length(joint_name)>=4
            joint.(joint_names{i,1}).(joint_name{end}).(joint_name{3})=joint_raw(1:data_length_capture,1+i);
        end
        
    end
    fieldname=fieldnames(joint);
    joint.joint_names=fieldname(5:end);
    

    
    %%
    [fx, P1, handle] = fftPlot(force_deci(:,10),plate.freq,'off',1);
    
    force_deci(:,2:7)=force_deci(:,2:7)-SubInfo.Offset1{sub_lgc};
    force_deci(:,8:13)=force_deci(:,8:13)-SubInfo.Offset2{sub_lgc};
    sens = [0.5 -0.5 1 0.6 -0.3 0.3 0.5 -0.5 1 0.6 -0.3 0.3]*5000/2^11;  % 2^N : daq-board
    for i = 1:12
        force_name = force_names{i};
        plate.(force_name(1:2)).(force_name(3)) = filtfilt(b_force, a_force,force_deci(:, i+1)*sens(i));
        plate30.(force_name(1:2)).(force_name(3)) = filtfilt(b2_force, a2_force,force_deci(:, i+1)*sens(i));
    end
    
    
    % cop
    center = [-0.3025 0.3025;0 0 ; 0 0];
    d = 0.116+0.009;
    for ii=1:2
        plate.(['C' num2str(ii)]).X=(plate.(['M' num2str(ii)]).Y+ ...
            d*plate.(['F' num2str(ii)]).X)./plate.(['F' num2str(ii)]).Z+center(1,ii);
        plate.(['C' num2str(ii)]).Y=-(-plate.(['M' num2str(ii)]).X- ...
            d*plate.(['F' num2str(ii)]).Y)./plate.(['F' num2str(ii)]).Z+center(2,ii);
        plate.(['C' num2str(ii)]).Z=zeros(length(plate.time),1)+center(3,ii);
    end
    
    try
        % segmentation
        if spi>=1 
            [subject,plate,capture]=phase_index(filename_data,subject,plate,capture,viz);
        end
        
    catch ME
        disp( getReport(ME, 'extended', 'hyperlinks', 'on' ) );
        warning(['can not find HS for' filename]);
        
    end
    
    if isempty(time_length)
        file_name_save=[filename_data];
        if mode_save
            
    %         save([filename '.mat'],'subject', 'capture', 'plate','capture30','plate30')
            save([file_name_save '.mat'],'subject', 'capture', 'plate','capture30','plate30','joint');
            fprintf('\nData %s.mat is saved!\n', filename);
        end
    else
        file_name_save=[filename_data '_' num2str(time_length) ];
        if mode_save
            
        %         save([filename '.mat'],'subject', 'capture', 'plate','capture30','plate30')
            save([file_name_save '.mat'],'subject', 'capture', 'plate','capture30','plate30','joint');
            fprintf('\nData %s.mat is saved!\n', filename);
        end
    end
end