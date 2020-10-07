function sample = selectSample(HS_ind, sub, kinetics, joints)
% sample one period of joints, GRF, CoM specified by heel strike index
% all samples are linearly interpolated to have points of 'num_points'

fprintf('(%s) ', mfilename)
fprintf('Sampling one joints, GRF and CoM trajectory ... ')

num_pts = 32;

% frame, init_xyz will be filled by writeURDF.m
sample = struct('sub_info', [], 'nframe', [], 'init_xyz', [], ...
    'time', [], 'pos', [], 'orn', [], 'lin_vel', [], ...
    'ang_vel', [], 'GRF', [], 'CoM', []);

start_ind = kinetics.HS(HS_ind);
end_ind = kinetics.HS(HS_ind+2)-1;

sample.sub_info = sub;

sample.time = linspace(kinetics.time(start_ind), kinetics.time(end_ind), ...
    num_pts)' - kinetics.time(start_ind);

% add x displacement caused by treadmill
sample.pos = pos_lin_erp(joints.pos);
sample.lin_vel = vel_lin_erp(joints.lin_vel);
sample.ang_vel = vel_lin_erp(joints.ang_vel);
sample.ang_vel.Rknee = sign(sample.ang_vel.Rknee(:,1)) ...
    .* vecnorm(sample.ang_vel.Rknee, 2, 2);
sample.ang_vel.Lknee = sign(sample.ang_vel.Lknee(:,1)) ...
    .* vecnorm(sample.ang_vel.Lknee, 2, 2);

% select one sample of joints pose
% convert quaternion on knee to angle (revolute joint)
sample.orn = joints_slerp(joints.orn);
sample.orn.Rknee = quat2angle(sample.orn.Rknee);
sample.orn.Lknee = quat2angle(sample.orn.Lknee);

% sample of GRF
% sample_GRF: a struct of (R/L) X, Y, Z, HS, TO
sample.GRF = sampleGRF(kinetics, start_ind, end_ind, num_pts);

% sample of CoM
sample.CoM = sampleCoM(sub, sample.time, sample.GRF);

fprintf('done!\n\n')


% ============ sub functions ============

    function sample = pos_lin_erp(data)
        % linearly interpolates pos data
        
        mat3 = zeros(num_pts, 3);       % X, Y, Z
        sample = struct('base', mat3, 'Rhip', mat3, 'Rknee', mat3, ...
            'Rankle', mat3, 'Lhip', mat3, 'Lknee', mat3, 'Lankle', mat3);
        field = ["base", "Rhip", "Rknee", "Rankle", "Lhip", "Lknee", "Lankle"];
        ori_ind = start_ind:end_ind;
        erp_ind = linspace(start_ind, end_ind, num_pts);
        for i = 1:numel(field)
            sample.(field(i))(:,1) = ...
                interp1(ori_ind, data.(field(i))(start_ind:end_ind,1), erp_ind) ...
                - data.(field(i))(start_ind, 1);
            sample.(field(i))(:,2) = ...
                interp1(ori_ind, data.(field(i))(start_ind:end_ind,2), erp_ind) ...
                - data.(field(i))(start_ind, 2);
            sample.(field(i))(:,3) = ...
                interp1(ori_ind, data.(field(i))(start_ind:end_ind,3), erp_ind);
        end
    end

    function sample = vel_lin_erp(data)
        % linearly interpolates linear/angular velocity data
        
        mat3 = zeros(num_pts, 3);       % X, Y, Z
        sample = struct('base', mat3, 'Rhip', mat3, 'Rknee', mat3, ...
            'Rankle', mat3, 'Lhip', mat3, 'Lknee', mat3, 'Lankle', mat3);
        field = ["base", "Rhip", "Rknee", "Rankle", "Lhip", "Lknee", "Lankle"];
        ori_ind = start_ind:end_ind;
        erp_ind = linspace(start_ind, end_ind, num_pts);
        for i = 1:numel(field)
            sample.(field(i))(:,1) = ...
                interp1(ori_ind, data.(field(i))(start_ind:end_ind,1), erp_ind);
            sample.(field(i))(:,2) = ...
                interp1(ori_ind, data.(field(i))(start_ind:end_ind,2), erp_ind);
            sample.(field(i))(:,3) = ...
                interp1(ori_ind, data.(field(i))(start_ind:end_ind,3), erp_ind);
        end
    end

    function sample = joints_slerp(joints)
        % sample 7 joints pose which is spherically linear-interpolated
        
        quats = zeros(num_pts, 4);
        sample = struct('base', quats, 'Rhip', quats, 'Rknee', quats, ...
            'Rankle', quats, 'Lhip', quats, 'Lknee', quats, 'Lankle', quats);
        sam_ind = linspace(start_ind, end_ind, num_pts);
        for i = 1:num_pts
            ind = floor(sam_ind(i));
            frac = sam_ind(i) - ind;
            sample.base(i,:) = slerpG(joints.base(ind,:), joints.base(ind+1,:), frac);
            sample.Rhip(i,:) = slerpG(joints.Rhip(ind,:), joints.Rhip(ind+1,:), frac);
            sample.Rknee(i,:) = slerpG(joints.Rknee(ind,:), joints.Rknee(ind+1,:), frac);
            sample.Rankle(i,:) = slerpG(joints.Rankle(ind,:), joints.Rankle(ind+1,:), frac);
            sample.Lhip(i,:) = slerpG(joints.Lhip(ind,:), joints.Lhip(ind+1,:), frac);
            sample.Lknee(i,:) = slerpG(joints.Lknee(ind,:), joints.Lknee(ind+1,:), frac);
            sample.Lankle(i,:) = slerpG(joints.Lankle(ind,:), joints.Lankle(ind+1,:), frac);
        end
    end

    function angles = quat2angle(quats)
        % convert quaternion to angle assuming a revolute joint
        angles = zeros(size(quats, 1), 1);
        for i = 1:numel(angles)
            angles(i) = sign(quats(1))*2*acos(quats(i,4));
        end
    end
end

    function quat = slerpG(p, q, t)
        % calculate quaternion slerp
        % https://en.wikipedia.org/wiki/Slerp
        dot = p*q';
        if dot < 0
            q = -q;
            dot = -dot;
        end
        th = acos(dot);
        if th == 0
            quat = p;
        else
            quat = (sin((1-t)*th)*p + sin(t*th)*q)/sin(th);
        end
    end

    function sample = sampleGRF(kinetics, start_ind, end_ind, num_pts)
        % sample one stride GRF from start to end
        % it's linearly interpolated to have points of num_pts
        
        sample = struct('R', struct('X',[],'Y',[],'Z',[],'HS',[],'TO',[]), ...
            'L', struct('X',[],'Y',[],'Z',[],'HS',[],'TO',[]));
        
        % interpolate of XYZ data
        interp_ind = linspace(1, end_ind-start_ind+1, num_pts);
        sample.R.X = interp1(kinetics.F2.X(start_ind:end_ind), interp_ind);
        sample.R.Y = interp1(kinetics.F2.Y(start_ind:end_ind), interp_ind);
        sample.R.Z = interp1(kinetics.F2.Z(start_ind:end_ind), interp_ind);
        sample.L.X = interp1(kinetics.F1.X(start_ind:end_ind), interp_ind);
        sample.L.Y = interp1(kinetics.F1.Y(start_ind:end_ind), interp_ind);
        sample.L.Z = interp1(kinetics.F1.Z(start_ind:end_ind), interp_ind);
        
        % assign HS/TO
        gap = (end_ind-start_ind)/(num_pts-1);
        HS_ind = find(kinetics.HS == start_ind);
        if kinetics.HS_leg(HS_ind) == 'R'
            sample.R.HS = 1;
            sample.R.TO = round((kinetics.TO(HS_ind)-start_ind)/gap);
            sample.L.HS = round((kinetics.HS(HS_ind+1)-start_ind)/gap);
            sample.L.TO = round((kinetics.TO(HS_ind-1)-start_ind)/gap);
            
        elseif kinetics.HS_leg(HS_ind) == 'L'
            sample.R.HS = round((kinetics.HS(HS_ind+1)-start_ind)/gap);
            sample.R.TO = round((kinetics.TO(HS_ind-1)-start_ind)/gap);
            sample.L.HS = 1;
            sample.L.TO = round((kinetics.TO(HS_ind)-start_ind)/gap);
        end
    end

    function sample = sampleCoM(sub_info, time, GRF)
        % sample the center of mass trajectori by integrating acceleration twice
        % enforce zero displacement at 50%, 100% twice
        
        acc = ([GRF.R.X; GRF.R.Y; GRF.R.Z] + [GRF.L.X; GRF.L.Y; GRF.L.Z])/sub_info.mass ...
            - [0, 0, 9.81]';
        half_ind = floor(numel(time)/2);
        acc_cum_a = cumtrapz(time(1:half_ind), acc(:,1:half_ind), 2);
        acc_cum_b = cumtrapz(time(half_ind+1:end), acc(:,half_ind+1:end), 2);
        vel = [acc_cum_a-mean(acc_cum_a,2), acc_cum_b-mean(acc_cum_b,2)];
        com = cumtrapz(time, vel, 2);
        sample = struct('X', com(1,:)', 'Y', com(2,:)', 'Z', com(3,:)');
    end

