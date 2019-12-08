clear
close all
clc

addpath('functions');

Tk = 0.001;

%% 1

tic
p = [0; 0; 0];
q = [0; 0; 0; 0; 0; 0];

[R_se, J, tmp] = getJacobian_spatial(q);
[R_se2, J2, tmp2] = getJacobian_1(q);
pos = R_se(1:3, 4) + R_se(1:3, 1:3) * [0; 0; 3*sqrt(2)];
distance = norm(pos - p);
dist_arr = [];

gain = 10;
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
    [R_se, J, tmp] = getJacobian_spatial(q);
    pos = R_se(1:3, 4) + R_se(1:3, 1:3) * [0; 0; 3*sqrt(2)];
    distance = norm(pos - p);
    dist_arr = [dist_arr; distance];
end
toc

%% 2
close all;
figure(2);
hold on;
grid on;
[R_se, J, x] = getJacobian_spatial(q);
plot3(x(:, 1), x(:, 2), x(:, 3));
[x_s, y_s, z_s] = sphere;
r = 0.1;
surf(x_s*r+p(1), y_s*r+p(2), z_s*r+p(3))
axis equal
xlabel('x(t)')
ylabel('y(t)')
zlabel('z(t)')
view(10, 10);