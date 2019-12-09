%% Dynamics
alpha = 0;
q = [0; 0; 0; 0; 0; 0];
dq = [0; 0; 0; 0; 0; 0];
[R_base_e, J, x] = getJacobian(q);
% [M,Mdot,C,Grav] = NE_matrix(q, dq);

%% Impedance control without inertial shaping
% TODO: setup xd / Rd
p = [0; -0.5; 0.8];

w = 0.2;

elapsedTime = 0;
tic
% while (elapsedTime < 3)
    elapsedTime =toc;
    p(1) = 0.5 * cos(w*elapsedTime);
    p(2) = 0.5 * sin(w*elapsedTime);
    
    p_base = inv(T_s0) * [p; 1];
    theta = atan(p_base(2)/(1.5*sqrt(2)-p_base(1)));
    T_base_EE = eye(4);
    T_base_EE(1, 4) = 1.5*sqrt(2) - cos(theta);
    T_base_EE(2, 4) = sin(theta);
    T_base_EE(1:3, 1:3) = [0, sin(theta), cos(theta);
        0, -cos(theta), -sin(theta);
        1, 0, 0]
    T_base_6 = T_base_EE * inv(T_6e); 
%     q = IRB1.ikine(T_base_6)
    % result
% end
