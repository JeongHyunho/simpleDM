function joints = simHumanoidJointsPose(sub, kin)
% calculate joints position/ orientation/ linear velocity/ angular velocity
% in quaternoion for every frames

fprintf('(%s) ', mfilename)
fprintf('Calculating joints pose in quaternions ...')

numFrame = numel(kin.time);
mat_3 = zeros(numFrame, 3);
mat_4 = zeros(numFrame, 4);

field = ["base", "Rhip", "Rknee", "Rankle", "Lhip", "Lknee", "Lankle"];

pos = struct('base', mat_3, 'Rhip', mat_3, 'Rknee', mat_3, ...
    'Ranke', mat_3, 'Lhip', mat_3, 'Lknee', mat_3, 'Lankle', mat_3);
orn = struct('base', mat_4, 'Rhip', mat_4, 'Rknee', mat_4, 'Ranke', mat_4, ...
    'Lhip', mat_4, 'Lknee', mat_4, 'Lankle', mat_4);
lin_vel = struct('base', mat_3, 'Rhip', mat_3, 'Rknee', mat_3, ...
    'Ranke', mat_3, 'Lhip', mat_3, 'Lknee', mat_3, 'Lankle', mat_3);
ang_vel = struct('base', mat_3, 'Rhip', mat_3, 'Rknee', mat_3, ...
    'Ranke', mat_3, 'Lhip', mat_3, 'Lknee', mat_3, 'Lankle', mat_3);

joints = struct('pos', pos, 'orn', orn', 'lin_vel', lin_vel, ...
    'ang_vel', ang_vel);

for i = 1:numFrame
    % base
    cent_trunk = 0.5*(marker3D(kin.V_R_Hip_JC,i) + marker3D(kin.V_L_Hip_JC,i));
    pos.base(i,:) = 0.5*(cent_trunk + marker3D(kin.STRN, i));
    X_trunk = (marker3D(kin.V_R_Hip_JC,i) - marker3D(kin.V_L_Hip_JC,i));
    Z_trunk = (marker3D(kin.C7, i) + marker3D(kin.STRN, i))/2 - ...
        (marker3D(kin.V_R_Hip_JC, i) + marker3D(kin.V_L_Hip_JC, i))/2;
    Z_trunk = Z_trunk - Z_trunk'*X_trunk*X_trunk/norm(X_trunk)^2;
    Y_trunk = cross(Z_trunk, X_trunk);
    Rotm_trunk = [X_trunk/norm(X_trunk), Y_trunk/norm(Y_trunk), ...
        Z_trunk/norm(Z_trunk)];
    
    % right hip
    pos.Rhip(i,:) = marker3D(kin.V_R_Hip_JC,i);
    X_Rthigh = marker3D(kin.R_Knee,i) - marker3D(kin.V_R_Knee_JC,i);
    Z_Rthigh = marker3D(kin.V_R_Hip_JC,i) - marker3D(kin.V_R_Knee_JC,i);
    X_Rthigh = X_Rthigh - X_Rthigh'*Z_Rthigh*Z_Rthigh/norm(Z_Rthigh)^2;
    Y_Rthigh = cross(Z_Rthigh, X_Rthigh);
    Rotm_Rthigh = [X_Rthigh/norm(X_Rthigh), Y_Rthigh/norm(Y_Rthigh), ...
        Z_Rthigh/norm(Z_Rthigh)];
    
    % right knee
    pos.Rknee(i,:) = marker3D(kin.V_R_Knee_JC,i);
    X_Rshank = X_Rthigh;
    Z_Rshank = marker3D(kin.V_R_Knee_JC,i) - marker3D(kin.V_R_Ankle_JC,i);
    Z_Rshank = Z_Rshank - Z_Rshank'*X_Rshank*X_Rshank/norm(X_Rshank)^2;
    Y_Rshank = cross(Z_Rshank, X_Rshank);
    Rotm_Rshank = [X_Rshank/norm(X_Rshank), Y_Rshank/norm(Y_Rshank), ...
        Z_Rshank/norm(Z_Rshank)];
    
    % right ankle
    pos.Rankle(i,:) = marker3D(kin.V_R_Hip_JC,i);
    X_Rfoot = marker3D(kin.R_Little,i) - marker3D(kin.R_Thumb,i);
    Y_Rfoot = marker3D(kin.V_R_Toe_Offset,i) - marker3D(kin.R_Heel,i);
    X_Rfoot = X_Rfoot - X_Rfoot'*Y_Rfoot*Y_Rfoot/norm(Y_Rfoot)^2;
    Z_Rfoot = cross(X_Rfoot, Y_Rfoot);
    Rotm_Rfoot = [X_Rfoot/norm(X_Rfoot), Y_Rfoot/norm(Y_Rfoot), ...
        Z_Rfoot/norm(Z_Rfoot)];
    
    % left hip
    pos.Lhip(i,:) = marker3D(kin.V_L_Hip_JC,i);
    X_Lthigh = -(marker3D(kin.L_Knee,i) - marker3D(kin.V_L_Knee_JC,i));
    Z_Lthigh = marker3D(kin.V_L_Hip_JC,i) - marker3D(kin.V_L_Knee_JC,i);
    X_Lthigh = X_Lthigh - X_Lthigh'*Z_Lthigh*Z_Lthigh/norm(Z_Lthigh)^2;
    Y_Lthigh = cross(Z_Lthigh, X_Lthigh);
    Rotm_Lthigh = [X_Lthigh/norm(X_Lthigh), Y_Lthigh/norm(Y_Lthigh), ...
        Z_Lthigh/norm(Z_Lthigh)];
    
    % left knee
    pos.Lknee(i,:) = marker3D(kin.V_L_Knee_JC,i);
    X_Lshank = X_Lthigh;
    Z_Lshank = marker3D(kin.V_L_Knee_JC,i) - marker3D(kin.V_L_Ankle_JC,i);
    Z_Lshank = Z_Lshank - Z_Lshank'*X_Lshank*X_Lshank/norm(X_Lshank)^2;
    Y_Lshank = cross(Z_Lshank, X_Lshank);
    Rotm_Lshank = [X_Lshank/norm(X_Lshank), Y_Lshank/norm(Y_Lshank), ...
        Z_Lshank/norm(Z_Lshank)];
    
    % left ankle
    pos.Lankle(i,:) = marker3D(kin.V_L_Hip_JC,i);
    X_Lfoot = -(marker3D(kin.L_Little,i) - marker3D(kin.L_Thumb,i));
    Y_Lfoot = marker3D(kin.V_L_Toe_Offset,i) - marker3D(kin.L_Heel,i);
    X_Lfoot = X_Lfoot - X_Lfoot'*Y_Lfoot*Y_Lfoot/norm(Y_Lfoot)^2;
    Z_Lfoot = cross(X_Lfoot, Y_Lfoot);
    Rotm_Lfoot = [X_Lfoot/norm(X_Lfoot), Y_Lfoot/norm(Y_Lfoot), ...
        Z_Lfoot/norm(Z_Lfoot)];
    
    % calculate joints angle in quaternion
    orn.base(i,:) = rotm2quatG(Rotm_trunk);
    orn.Rhip(i,:) = rotm2quatG(Rotm_trunk'*Rotm_Rthigh);
    orn.Rknee(i,:) = rotm2quatG(Rotm_Rthigh'*Rotm_Rshank);
    orn.Rankle(i,:) = rotm2quatG(Rotm_Rshank'*Rotm_Rfoot);
    orn.Lhip(i,:) = rotm2quatG(Rotm_trunk'*Rotm_Lthigh);
    orn.Lknee(i,:) = rotm2quatG(Rotm_Lthigh'*Rotm_Lshank);
    orn.Lankle(i,:) = rotm2quatG(Rotm_Lshank'*Rotm_Lfoot);
end

% considering treadmil
tread_Y = sub.speed * kin.time;
for i = 1:numel(field)
    pos.(field(i))(:,2) = pos.(field(i))(:,2) + tread_Y;
end

n = 1;
dim = 1;
for i = 1:numel(field)
    % linear velocity
    lin_vel.(field(i)) = kin.freq * diff(pos.(field(i)), n, dim);
    lin_vel.(field(i)) = [lin_vel.(field(i)); lin_vel.(field(i))(end,:)];
    
    % angular velocity
    for j = 1:numFrame-1
        [axis, angle] = quatDiff2axang(orn.(field(i))(j,:), ...
            orn.(field(i))(j+1,:));
        ang_vel.(field(i))(j,:) = angle * kin.freq * axis;
    end
end

joints.pos = pos;
joints.orn = orn;
joints.lin_vel = lin_vel;
joints.ang_vel = ang_vel;

fprintf('done!\n\n')
end

function point = marker3D(s, frame)
% marker points [X, Y, Z] of a frame

point = [s.X(frame), s.Y(frame), s.Z(frame)]';
end

function quatG = rotm2quatG(rotm)
% calculate quaternions from rotation matrix (axis u, angle th)
% quatG = [u(1)*sin(th/2), u(2)*sin(th/2), u(3)*sin(th/2), cos(th/2)]

axang = rotm2axang(rotm);
axis = axang(1:3);
th = axang(4);
quatG = [sin(th/2)*axis, cos(th/2)];
end

function [axis, angle] = quatDiff2axang(q2, q1)
% (q2 - q1) to axis and angle
% (q2 - q1) equals to conj(q1)*q2
r1 = q1(4);
v1 = q1(1:3);
r2 = q2(4);
v2 = q2(1:3);
dq = [r1*v2 - r2*v1 - cross(v1,v2), r1*r2 + v1*v2'];

angle = 2*acos(dq(4));
axis = dq(1:3)/sin(angle/2);
end