clear
close all
clc

addpath('functions');

Tk = 0.001;

%% 1

tic
p = [0; 0; 0];
q = [0; 0; 0; 0; 0; 0];

[R_se, J, tmp] = getJacobian(q);
[R_se2, J2, tmp2] = getJacobian_1(q);
pos = R_se(1:3, 4) + R_se(1:3, 1:3) * [0; 0; 3*sqrt(2)];
distance = norm(pos - p);
dist_arr = [];

gain = 100;
while ( abs(distance) > 0.0001 )    
    % control
    mat = [0, -3*sqrt(2), 0;
        3*sqrt(2), 0, 0;
        0, 0, 0];
    J1 = [eye(3) -R_se(1:3, 1:3)* mat *R_se(1:3, 1:3)';
        zeros(3) eye(3)] *J;
    
    q_dot = -pinv(J1) * gain * [(pos - p); zeros(3, 1)];
    q = q + q_dot * Tk;
    
% %     visualization
%     figure(1);
%     hold on;
%     grid on;
%     plot3(x(:, 1), x(:, 2), x(:, 3));
%     [x_s, y_s, z_s] = sphere;
%     r = 0.1;
%     surf(x_s*r+p(1), y_s*r+p(2), z_s*r+p(3))
%     axis equal
%     xlabel('x(t)')
%     ylabel('y(t)')
%     zlabel('z(t)')
%     view(10, 10);
    
    % update
    [R_se, J, tmp] = getJacobian(q);
    pos = R_se(1:3, 4) + R_se(1:3, 1:3) * [0; 0; 3*sqrt(2)];
    distance = norm(pos - p);
    dist_arr = [dist_arr; distance];
end
toc

%% 2
figure(2);
hold on;
grid on;
% [R_se, J, x] = getJacobian(q);
plot3(x(:, 1), x(:, 2), x(:, 3));
[x_s, y_s, z_s] = sphere;
r = 0.1;
surf(x_s*r+p(1), y_s*r+p(2), z_s*r+p(3))
axis equal
xlabel('x(t)')
ylabel('y(t)')
zlabel('z(t)')
view(10, 10);

%% 3
tic
p = [0.5; 0.0; 0];
q = [0; 0; 0; 0; 0; 0];

[R_se, J, tmp] = getJacobian(q);
distance = norm(R_se2(1:3, 4) - p);
dist_arr = [];
x_arr = [];
w_arr = [];

w = 3;
[x_s, y_s, z_s] = sphere;
r = 0.1;

% control parameters
gain = 1;
% weight = diag([25, 5, 1]);

elapsedTime = 0.0;
tic
while (elapsedTime < 10)
% while (true)
    % jacobian
    elapsedTime = toc;
    p(1) = 0.5 * cos(w * elapsedTime);
    p(2) = 0.5 * sin(w * elapsedTime);
    p_dot(1) = -0.5 * w * sin(w * elapsedTime);
    p_dot(2) = 0.5 * w * cos(w * elapsedTime);
    p_dot(3) = 0;
    
    J1 = [J2(1:3, :); zeros(3, 6)];
    P1 = eye(6) - pinv(J1) * J1;
    J2 = [zeros(3) eye(3)] * J;
%     P2 = P1 - pinv(J2 * P1) * J2 * P1;
    
    % control (Task1 - P control)
    q1_dot = -pinv(J1) * gain * [(R_se2(1:3, 4) - p); zeros(3, 1)];
    q2_dot = q1_dot + pinv(J2 * P1) * (0 - J2 * q1_dot);
%     q3_dot = q2_dot + 
    
%     q = q + q1_dot * Tk;
    q = q + q2_dot * Tk;
    
    % update state
    [R_se2, J2, x] = getJacobian(q);
    distance = norm(R_se2(1:3, 4) - p);
    dist_arr = [dist_arr; distance];
    x_arr = [x_arr R_se2(1:3, 4)];
end
toc

%% 4