clc; clear; close all;
startup_rvc;

%% Simulation Setup
Tk = 0.001;

%% Robot Setup
alpha = [0; -pi/2; 0;     pi/2;  -pi/2; pi/2];
a =     [0; 0.160; 0.980; 0.150; 0;    0];  
offset = [0; -pi/2; 0;     0;     0;    0];
d =     [0;   0;    0;   -0.860; 0;     0]; 

for i = 1:6
    L(i) = Link('d', d(i), 'a', a(i), 'alpha', alpha(i), 'offset', offset(i), 'modified'); 
end
IRB = SerialLink(L);

T_s0 = Trans('x', 3) * Trans('y', 3) * Rot('z', pi*5/4) * Trans('z', 0.520);
T_6e = Rot('x', pi) * Trans('x', -0.120) * Trans('z', 0.303);


%% #1
clc;
% q(:,1) = [0; 0; 0; 0; 0; 0];
q = [0; 0; 0; 0; 0; 0];

p = [0, 0, 0];
p_0 = inv(T_s0) * [p'; 1];
% [R_se, J, tmp] = getJacobian(q(:,1));
[R_se, J, tmp] = getJacobian(q);
pos = R_se(1:3, 4) + R_se(1:3, 1:3) * [0; 0; 3*sqrt(2)];
distance = norm(pos - p_0(1:3));
dist_arr = [];

gain = 50;

i = 1;
while (abs(distance) > 1e-4)
    % control
    mat = [0, -3*sqrt(2), 0;
        3*sqrt(2), 0, 0;
        0, 0, 0];
    J1 = [eye(3) -R_se(1:3, 1:3)* mat *R_se(1:3, 1:3)';
        zeros(3) eye(3)] *J;
    i = i + 1;
    q_dot = -pinv(J1) * gain * [(pos - p_0(1:3)); zeros(3, 1)];
    [det(J1); q_dot]
%     q = [q q(:,i) + q_dot * Tk];
    q = q + q_dot * Tk;
    i = i+1;
    
    % update
%     [R_se, J, tmp] = getJacobian(q(:,i));
    [R_se, J, tmp] = getJacobian(q);
    pos = R_se(1:3, 4) + R_se(1:3, 1:3) * [0; 0; 3*sqrt(2)];
    distance = norm(pos - p_0(1:3));
    dist_arr = [dist_arr; distance];
end

%% Plot

% Robot Animation
for i = 1:size(q,2)
    IRB.plot(q(:,i)')
end

%% #3