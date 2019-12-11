%% Dynamics
alpha = 0;
q = [0; 0; 0; 0; 0; 0];
dq = [0; 0; 0; 0; 0; 0];
[R_base_e, J, x] = getJacobian(q);
% [M,Mdot,C,Grav] = NE_matrix(q, dq);

% %% Impedance control without inertial shaping
% T_s0 = Trans('x', 1.5) * Trans('y', -1.5) * Rot('z', pi*3/4) * Trans('z', 0.520);
% 
% ugv_center = [-0.75 * cos(pi/4); 0.75 * sin(pi/4); 0.8];
% ugv_global = [ugv_center(1) + 0.25; ugv_center(2); 0.8];
% 
% w = 1;
% gain_v = 10;
% gain_w = 10;
% elapsedTime = 0;
% q = [];
% qi = zeros(7, 1);
% 
% [R_se, J, tmp] = getJacobian(qi);
% e = zeros(6, 1); de = zeros(6, 1); % x error
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
%     
%     aa_tmp = rotm2axang(R_se(1:3, 1:3)' * EE_base(1:3, 1:3));
%     del_aa = aa_tmp(1:3) * aa_tmp(4);
%     
%     x_error(1:3) = R_se(1:3, 4) - EE_base(1:3, 4);
% %     e(4:6) = del_aa;1
%     x_error(4:6) = zeros(3,1);
%     
%     % Desired dynamics: M(ddq) + C(dq) + J^TKe = 0
%     Md = diag([5, 5, 5, 5, 5, 5]);
%     Kd = diag([20000, 20000, 20000, 20000, 20000, 20000]);
%     Cd = 2 * sqrt(Md * Kd); % critical damping
% %     J(:,3:6) = zeros(6,3);
%     
%     % dynamic integration (semi explicit euler integration)
%     de_next = de - Tk * pinv(Md * J) * (Cd * J * de + Kd * x_error);
%     
%     ei = ei + [de_next * Tk; 0];
%     qi = 
%     q = [q qi];
%     [R_se, J, tmp] = getJacobian(qi);
%     dq = dq_next;
% end

%% Impedance control without inertial shaping
T_s0 = Trans('x', 1.5) * Trans('y', -1.5) * Rot('z', pi*3/4) * Trans('z', 0.520);

ugv_center = [-0.75 * cos(pi/4); 0.75 * sin(pi/4); 0.8];
ugv_global = [ugv_center(1) + 0.25; ugv_center(2); 0.8];

w = 1;
elapsedTime = 0;
q = [];
qi = zeros(7, 1);

e = zeros(6, 1); de = zeros(6, 1);
tic
while (elapsedTime < 15)
    elapsedTime =toc;
    % UAV pose update
    ugv_global = [ugv_center(1) + 0.25 * cos(w * elapsedTime);
        ugv_center(2) + 0.25 * sin(w * elapsedTime);
        0.8];
    ugv_base = inv(T_s0) * [ugv_global ; 1];
    
    % end-effector desired position
    theta = atan(-ugv_global(2) / ugv_global(1));
    EE_global = eye(4);
    EE_global(1:3, 4) = [1.5 * cos(theta); -1.5 * sin(theta); 0.8];
    EE_global(1:3, 1:3) = [0, sin(theta), -cos(theta);
        0, cos(theta), sin(theta)
        1, 0, 0];
    EE_base = inv(T_s0) * EE_global;
    L6_base = EE_base * inv(T_6e);
    qd = [IRB_link.ikine(L6_base)'; 0];
    
    e = qi(1:6) - qd(1:6);
    % Desired dynamics: M(dde) + C(de) + Ke = 0
    Md = diag([5, 5, 5, 5, 5, 5]);
    Kd = diag([20000, 20000, 20000, 20000, 20000, 20000]);
    Cd = 2 * sqrt(Md * Kd); % critical damping
    
    % dynamic integration (semi explicit euler integration)
    de_next = Tk * inv(Md)* (Md/Tk * de - Cd * de - Kd * e);
    e_next = Tk * de_next + e;
    
    qi = [e_next + qd(1:6); 0];
    q = [q qi];
    de = de_next;
end