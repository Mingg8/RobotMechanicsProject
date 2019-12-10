%% Dynamics
alpha = 0;
q = [0; 0; 0; 0; 0; 0];
dq = [0; 0; 0; 0; 0; 0];
[R_base_e, J, x] = getJacobian(q);
% [M,Mdot,C,Grav] = NE_matrix(q, dq);

%% Impedance control without inertial shaping
T_s0 = Trans('x', 1.5) * Trans('y', -1.5) * Rot('z', pi*3/4) * Trans('z', 0.520);

ugv_center = [-0.75 * cos(pi/4); 0.75 * sin(pi/4); 0.8];
ugv_global = [ugv_center(1) + 0.25; ugv_center(2); 0.8];

w = 1;
gain_v = 10;
gain_w = 10;
elapsedTime = 0;
q = [];
qi = zeros(7, 1);

[R_se, J, tmp] = getJacobian(qi);
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
    EE_global(1:3, 4) = [3 * cos(theta); -3 * sin(theta); 0.8];
    EE_global(1:3, 1:3) = [0, sin(theta), -cos(theta);
        0, cos(theta), sin(theta)
        1, 0, 0];
    EE_base = inv(T_s0) * EE_global;
    
    del_aa = rotm2axang(R_se' * EE_base(1:3, 1:3));
    
    e(1:3) = R_se(1:3, 4) - EE_base(1:3, 4);
    e(4:6) = del_aa;
    
    % Desired dynamics: M(dde) + C(de) + Ke = 0
    Md = diag([5, 5, 5, 5, 5, 5]);
    Kd = diag([20, 20, 200, 200, 200, 200]);
    Cd = 2 * sqrt(Md * Kd); % critical damping
    
    % dynamic integration (semi explicit euler integration)
    de_next = Tk * inv(Md)* (Md/Tk * de - Cd * de - Kd * e);
    e_next = Tk * de_next + e;
    
    qi = [e_next + qd(1:6); 0];
    q = [q qi];
    de = de_next;
end
