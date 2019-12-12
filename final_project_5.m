%% Dynamics
alpha = 0;
q = [0; 0; 0; 0; 0; 0];
dq = [0; 0; 0; 0; 0; 0];
[R_base_e, J, x] = getJacobian(q);
% [M,Mdot,C,Grav] = NE_matrix(q, dq);

%% Impedance control without inertial shaping
T_s0 = Trans('x', -1.5) * Trans('y', 1.5) * Rot('z', -pi*1/4) * Trans('z', 0.520);

ugv_center = [0.75 * cos(pi/4); -0.75 * sin(pi/4); 1];
ugv_global = [ugv_center(1) + 0.25; ugv_center(2); 1];
ugv_global_arr = [];

w = 1;
gain_v = 10;
gain_w = 10;
elapsedTime = 0;
q = [-2.2142; -2.0737; 0.7152; 1.2337; -1.6420; -2.9414; 0];
qi = q;

[T_0e, J, tmp] = getJacobian(qi);
e = zeros(6, 1); 
dq = zeros(6, 1);
ddesired = zeros(6, 1); % x error
EE_base_prev = eye(4);
ddesired_prev = zeros(6, 1);
dxr = zeros(6, 1); xr = T_0e(1:3, 4); rot_r = T_0e(1:3, 1:3);
xr_prev = xr; rot_r_prev = rot_r;

e_ad = zeros(6, 1);

Md = diag([5, 5, 5, 5, 5, 5]);
Kd = diag([200, 200, 200, 200, 200, 200]);
Cd = 2 * sqrt(Md * Kd); % critical damping

Mi = Md;
Ki = Kd * 10;
Ci = 2 * sqrt(Mi * Ki);

tic
while (elapsedTime < 15)
    elapsedTime =toc;
    % UAV pose update
    ugv_global = [ugv_center(1) + 0.25 * cos(w * elapsedTime);
        ugv_center(2) + 0.25 * sin(w * elapsedTime);
        0.8];
    ugv_global_arr = [ugv_global_arr ugv_global];
    ugv_base = inv(T_s0) * [ugv_global ; 1];
    
    % end-effector desired position
    theta = atan(-ugv_global(2) / ugv_global(1));
    EE_global = eye(4);
    EE_global(1:3, 4) = [-1.5 * cos(theta); 1.5 * sin(theta); 1];
    EE_global(1:3, 1:3) = [0, -sin(theta), cos(theta);
        0, -cos(theta), -sin(theta)
        1, 0, 0];
    EE_base = inv(T_s0) * EE_global;
    
    % admittance dynamics
    aa_tmp = rotm2axang(rot_r(1:3, 1:3) * EE_base(1:3, 1:3)');
    del_aa = aa_tmp(1:3) * aa_tmp(4);
    e(1:3) = xr - EE_base(1:3, 4);
    e(4:6) = del_aa;
    
%     compute x_desired dot
    ddesired(1:3) = (EE_base(1:3, 4) - EE_base_prev(1:3, 4)) / Tk;
    aa_tmp = rotm2axang(EE_base(1:3, 1:3) * EE_base_prev(1:3, 1:3)');
    ddesired(4:6) = aa_tmp(1:3) * aa_tmp(4);
    dddesired = (ddesired - ddesired_prev) / Tk;
    
    ddesired_prev = ddesired;
    EE_base_prev = EE_base;
    
    % get contact force
    f_contact = zeros(6, 1);
        
    % Desired dynamics: M(dde) + C(de) + Ke = 0
    % Compute reference x (xr, rot_r)
    f = f_contact + Md * dddesired + Cd * ddesired - Kd * e;
    dxr_next = dxr + Tk * inv(Md) * (f - Cd * dxr);
    xr = xr + Tk * dxr_next(1:3);
    rot_delta = axang2rotm([dxr_next(4:6)'/norm(dxr_next(4:6)) norm(dxr_next(4:6))*Tk]);
    rot_r = rot_delta * rot_r;
        
    % impedance dynamics
    er(1:3) = T_0e(1:3, 4) - xr;
    aa_tmp = rotm2axang(T_0e(1:3, 1:3) * rot_r');
    er(4:6) = aa_tmp(1:3) * aa_tmp(4);
%     
%     dxr(1:3) = (xr - xr_prev) / Tk;
%     aa_tmp = rotm2axang(rot_r * rot_r_prev');
%     dxr(4:6) = aa_tmp(1:3) * aa_tmp(4);
    
%     dynamic integration (semi explicit euler integration)
    dq_next = dq - Tk * inv(Mi) * J' * (Ci * (J * dq - dxr_next) + Ki * er');
    qi = qi + [dq_next * Tk; 0];
    q = [q qi];
    [T_0e, J, tmp] = getJacobian(qi);
    
    dq = dq_next;
    rot_r_prev = rot_r;
    xr_prev = xr;
    rot_r_prev = rot_r;
    dxr = dxr_next;
end
