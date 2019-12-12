%% Dynamics
alpha = 0;
q = zeros(7,1);
dq = [0; 0; 0; 0; 0; 0];
% [R_base_e, J, x] = getJacobian(q);
% [M,Mdot,C,Grav] = NE_matrix(q, dq);

%% Impedance control without inertial shaping
T_s0 = Trans('x', 1.5) * Trans('y', -1.5) * Rot('z', pi*3/4) * Trans('z', 0.520);

ugv_center = [-0.5 * cos(pi/4); 0.5 * sin(pi/4); 0.8];
ugv_global = [ugv_center(1) + 0.25; ugv_center(2); 0.8];
ugv_global_arr = [];

w = 5;
elapsedTime = 0;
q = [-2.89339565357794;-2.05812875006110;0.480179789872066;-5.87172911886179;-1.56424063858637;-9.42763859597744;0];
qi = q;

[T_0e, J, tmp] = getJacobian(qi);
e = zeros(6, 1); 
dq = zeros(6, 1);
ddesired = zeros(6, 1); % x error
ddesired_prev = ddesired;
EE_base_prev = eye(4);
R_se_prev = eye(4);
x_wall = zeros(2, 1); n_wall = zeros(2, 1);
fe = zeros(6, 1);
tic
k = 0;

Md = diag([5, 5, 5, 5, 5, 5]);
Kd = diag([100, 100, 100, 100, 100, 100]);
Cd = 2 * sqrt(Md * Kd); % critical damping

i = 1;
while (elapsedTime < 15)
    elapsedTime = toc;
%     UAV pose update
    ugv_global = [ugv_center(1) + 0.1 * cos(w * elapsedTime);
        ugv_center(2) + 0.1 * sin(w * elapsedTime);
        0.8];

    ugv_global_arr = [ugv_global_arr ugv_global];
    ugv_base = inv(T_s0) * [ugv_global ; 1];
    
%     end-effector desired position
    theta = atan(-ugv_global(2) / ugv_global(1));
    EE_global = eye(4);
    EE_global(1:3, 4) = [1 * cos(theta); -1 * sin(theta); 0.8];
    EE_global(1:3, 1:3) = [0, sin(theta), -cos(theta);
        0, cos(theta), sin(theta)
        1, 0, 0];
    EE_base = inv(T_s0) * EE_global;
    
    aa_tmp = rotm2axang(T_0e(1:3, 1:3)*EE_base(1:3, 1:3)');
    del_aa = aa_tmp(1:3) * aa_tmp(4);
    
    e(1:3) =  T_0e(1:3, 4) - EE_base(1:3, 4);
    e(4:6) = del_aa;
    
%     compute x_desired dot
    if (i == 1)
        EE_base_prev = EE_base;
    end
    ddesired(1:3) = (EE_base(1:3, 4) - EE_base_prev(1:3, 4)) / Tk;
    aa_tmp = logm(EE_base(1:3, 1:3) * EE_base_prev(1:3, 1:3)');
    ddesired(4:6) = [-aa_tmp(2,3), aa_tmp(1,3), -aa_tmp(1,2)]'/Tk;
    
%     Desired dynamics: M(ddq) + C(dq) + J^TKe = 0    
%     dynamic integration (semi explicit euler integration)
    dq_next = dq - Tk * inv(Md) * J' * (Cd * (J * dq - ddesired) + Kd * e);
    qi = qi + [dq_next * Tk; 0];
    q_sens = q_sens + (rand(1)-0.5) / 180.0 * pi;
    q = [q qi];
    
    dq = dq_next;
    EE_base_prev = EE_base;
    T_0e_prev = T_0e;
    [T_0e, J, tmp] = getJacobian(qi);
    
    % calculate fe
    T_se = T_s0 * T_0e;
    p_ee = T_se(1:2, 4);
    theta = atan(p_ee(2)/p_ee(1));
    x_wall(1) = 1 * cos(theta);
    x_wall(2) = 1 * sin(theta);
    n_wall = [cos(theta); sin(theta)];
    
    if dot(p_ee - x_wall, n_wall) < 0
        k = k + 1;
        fe = [- 500 * dot(p_ee - x_wall, n_wall) * n_wall];
        fe = T_s0(1:3, 1:3)' * [fe; 0];
        fe = [fe; zeros(3,1)];
        
    else
        fe = zeros(6,1);
    end
    i = i + 1;
end

%% Impedance control without inertial shaping
% T_s0 = Trans('x', 1.5) * Trans('y', -1.5) * Rot('z', pi*3/4) * Trans('z', 0.520);
% 
% ugv_center = [-0.75 * cos(pi/4); 0.75 * sin(pi/4); 0.8];
% ugv_global = [ugv_center(1) + 0.25; ugv_center(2); 0.8];
% 
% w = 1;
% elapsedTime = 0;
% q = [];
% qi = zeros(7, 1);
% 
% e = zeros(6, 1); de = zeros(6, 1);
% tic
% while (elapsedTime < 15)
%     elapsedTime =toc;
%     % UAV pose update
%     ugv_global = [ugv_center(1) + 0.25 * cos(w * elapsedTime);
%         ugv_center(2) + 0.25 * sin(w * elapsedTime);
%         0.8];
%     ugv_base = inv(T_s0) * [ugv_global ; 1];
%     
%     % end-effector desired position
%     theta = atan(-ugv_global(2) / ugv_global(1));
%     EE_global = eye(4);
%     EE_global(1:3, 4) = [1.5 * cos(theta); -1.5 * sin(theta); 0.8];
%     EE_global(1:3, 1:3) = [0, sin(theta), -cos(theta);
%         0, cos(theta), sin(theta)
%         1, 0, 0];
%     EE_base = inv(T_s0) * EE_global;
%     L6_base = EE_base * inv(T_6e);
%     qd = [IRB_link.ikine(L6_base)'; 0];
%     
%     e = qi(1:6) - qd(1:6);
%     % Desired dynamics: M(dde) + C(de) + Ke = 0
%     Md = diag([5, 5, 5, 5, 5, 5]);
%     Kd = diag([20000, 20000, 20000, 20000, 20000, 20000]);
%     Cd = 2 * sqrt(Md * Kd); % critical damping
%     
%     % dynamic integration (semi explicit euler integration)
%     de_next = Tk * inv(Md)* (Md/Tk * de - Cd * de - Kd * e);
%     e_next = Tk * de_next + e;
%     
%     qi = [e_next + qd(1:6); 0];
%     q = [q qi];
%     de = de_next;
% end