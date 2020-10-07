% WalkingDataEncode(file_name, [10 30], save_mode, viz);



% Info = strsplit(filename,'_');
% subject.subjN = Info{1};
% subject.Walking_speed = Info{2};
indx=find(strcmpi(SubInfo.subjN,subject.subjN));
Wspeed=SubInfo.(Info{2});

file_name_imu=[subject.subjN '\' subject.subjN '_v' num2str(round(Wspeed,2)) '.csv'];

%%
id = fopen(file_name_imu);
for ii=1:200
fgetl(id);
end
imu_names_raw = strsplit(fgetl(id), ',')';

raw_data=dlmread(file_name_imu,',',201,0);
end_line=min(find(raw_data(2:end)==0));
raw_data=raw_data(1:end_line,:);
IMU_raw = struct('step_time',[],'step_acc',[],'freq',[],'cut_off_freq',[]);
IMU_raw.time=raw_data(:,1);
IMU_raw.freq=(IMU_raw.time(2)-IMU_raw.time(1))^-1;

IMU_raw.imu_names=[];
for ii=1:length(imu_names_raw)/2
    names=strsplit(imu_names_raw{ii*2,1});
    if length(names{3})==1
        continue
    end
    IMU_raw.imu_names{str2double(names{1}(1)),1}=(names{1}(3:end-1));
    IMU_raw.(names{1}(3:end-1)).(names{2}).unit=(names{5}(2:end-1));
    IMU_raw.(names{1}(3:end-1)).(names{2}).(names{3}(end))=raw_data(:,ii*2);
end
imu_names=IMU_raw.imu_names;
IMU10=IMU_raw;
IMU30=IMU_raw;

%% filtering

[b_imu, a_imu] = butter(5, cut_off_freq(1)/IMU_raw.freq);
[b2_imu,a2_imu]= butter(5, cut_off_freq(2)/IMU_raw.freq);
IMU10.cut_off_freq = cut_off_freq(1);
IMU30.cut_off_freq = cut_off_freq(2);
%%
DATAi={'Acc','Gyro','Mag'};
AXII='XYZ';
Axii='xyz';
for ii=1:3
    for datai=1:3
        for axii=1:3
            IMU10.(imu_names{ii,1}).(DATAi{datai}).(AXII(axii))= ...
                filtfilt(b_imu,a_imu, IMU_raw.(imu_names{ii,1}).(DATAi{datai}).(AXII(axii)));
            IMU30.(imu_names{ii,1}).(DATAi{datai}).(AXII(axii))= ...
                filtfilt(b2_imu,a2_imu, IMU_raw.(imu_names{ii,1}).(DATAi{datai}).(AXII(axii)));
        end
    end
end

