% TODO: handle singular
clear
close all
clc

addpath('functions');

Tk = 0.001;

%% #3
p = [0.5; 0.0; 0];
q = [0; 0; 0; 0; 0; 0];

[R_se, J, tmp] = getJacobian(q);
pos = R_se(1:3, 4) + R_se(1:3, 1:3) * [0; 0; 3*sqrt(2)];

% for debugging
distance = norm(pos - p);
dist_arr = [];
x_arr = [];
w_arr = [];


w = 0.2;
[x_s, y_s, z_s] = sphere;
r = 0.1;

% control parameters
gain = 10;
% weight = diag([25, 5, 1]);

elapsedTime = 0.0;

tic
while (elapsedTime < 3.0)
    elapsedTime = toc;
    p(1) = 0.5 * cos(w * elapsedTime);
    p(2) = 0.5 * sin(w * elapsedTime);
    p_dot(1) = -0.5 * w * sin(w * elapsedTime);
    p_dot(2) = 0.5 * w * cos(w * elapsedTime);
    p_dot(3) = 0;

    % control
    mat = [0, -3*sqrt(2), 0;
        3*sqrt(2), 0, 0;
        0, 0, 0];
    J1 = [eye(3) -R_se(1:3, 1:3)* mat *R_se(1:3, 1:3)';
        zeros(3) eye(3)] *J;
    P1 = eye(6) - pinv(J1) * J1;
    J2 = [zeros(3) eye(3)] * J; % 3 by 6
    P2 = P1 - pinv(J2 * P1) * J2 * P1;
    J3 = [1 0 0 0 0 0 ];
    P3 = P2 - pinv(J3 * P2) * J3 * P2;
    
    r1_dot = gain * [(pos - p); zeros(3, 1)];
    r2_dot = zeros(3, 1);
    r3_dot = 0;
    
    q1_dot = -pinv(J1) * gain * [(pos - p); zeros(3, 1)];
   % J2 * P1 singular!!
   if (det(J2 * P1 * P1' * J2') < 1e-3)
       q2_dot = q1_dot + (P1' * J2') * (r2_dot - J2 * q1_dot);
%        "singular"
   else
       q2_dot = q1_dot + pinv(J2 * P1) * (r2_dot - J2 * q1_dot);
   end
   
   if (det(J3 * P2 * P2' * J3') < 1e-3)
        q3_dot = q2_dot + (J3 * P2)' * (r3_dot - J3 * q2_dot);
   else
        q3_dot = q2_dot + pinv(J3 * P2) * (r3_dot - J3 * q2_dot);
   end
    
    q = q + q3_dot * Tk;
    
    % update
    [R_se, J, tmp] = getJacobian(q);
    pos = R_se(1:3, 4) + R_se(1:3, 1:3) * [0; 0; 3*sqrt(2)];
    
    % for debugging
    distance = norm(pos - p);
    dist_arr = [dist_arr; distance];
    x_arr = [x_arr pos];
end
toc