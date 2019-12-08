clear
close all
clc

addpath('functions');

Tk = 0.001;

%% Dynamics
q = [0; 0; 0; 0; 0; 0];
[R, J, x] = getJacobian(q);
[M,Mdot,C,Grav] = NE_matrix(q, dq);

%% Impedance control without inertial shaping
% TODO: setup xd / Rd
qd = zeros(3,1); % IK
e = q - qd;
prev_e = e;
de = 0
prev_de = de;

while (true)
    
    % result
    q = qd + e;
end
