%% Dynamics
alpha = 0;
q = [0; 0; 0; 0; 0; 0];
dq = [0; 0; 0; 0; 0; 0];
[R_base_e, J, x] = getJacobian(q);
% [M,Mdot,C,Grav] = NE_matrix(q, dq);

%% Impedance control without inertial shaping
ugv_center = [0.75 * cos(pi/4); 0.75 * sin(pi/4); 0.8];
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
    theta = atan(ugv_global(2) / ugv_global(1));
    EE_global = eye(4);
    EE_global(1:3, 4) = [3 * cos(theta); 3 * sin(theta); 0.8];
    EE_global(1:3, 1:3) = [0, -sin(theta), -cos(theta);
        0, cos(theta), -sin(theta)
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
