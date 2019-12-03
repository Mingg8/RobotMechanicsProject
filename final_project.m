clear all
close all
clc

Tk = 0.001;

%% 1
tic
p = [0; 0.3; 0];
q = [0; 0; 0; 0; 0; 0];
R_so = Trans('x', 3) * Trans('y', 3) * Rot('z', pi*5/4) * Trans('z', 0.520);
R_s1 = R_so * Rot('z', q(1));
R_s2 = R_s1 * Rot('x', -pi/2) * Trans('x', 0.160) * Rot('z', -pi/2+q(2));
R_s3 = R_s2 * Trans('x', 0.980) * Rot('z', q(3));
R_s4 = R_s3 * Rot('x', pi/2) * Trans('x', 0.150) * Rot('z', q(4)) * Trans('z', -0.860);
R_s5 = R_s4 * Rot('x', -pi/2) * Rot('z', q(5));
R_s6 = R_s5 * Rot('x', pi/2) * Rot('z', q(6));
R_se = R_s6 * Rot('x', pi) * Trans('x', -0.120) * Trans('z', 0.303);

distance = norm(R_se(1:3, 4) - p) - 3 * sqrt(2);

J = zeros(3, 6);
gain = 0.5;
while ( abs(distance) > 0.0001 )
    % jacobian
    J(:, 1) = cross(R_s1(1:3, 3), R_se(1:3, 4) - R_s1(1:3, 1));
    J(:, 2) = cross(R_s2(1:3, 3), R_se(1:3, 4) - R_s2(1:3, 4));
    J(:, 3) = cross(R_s3(1:3, 3), R_se(1:3, 4) - R_s3(1:3, 4));
    J(:, 4) = cross(R_s4(1:3, 3), R_se(1:3, 4) - R_s4(1:3, 4));
    J(:, 5) = cross(R_s5(1:3, 3), R_se(1:3, 4) - R_s5(1:3, 4));
    J(:, 6) = cross(R_s6(1:3, 3), R_se(1:3, 4) - R_s6(1:3, 4));
    
    % control
    q_dot = gain * pinv(J) * (p - R_se(1:3, 4));
    q = q + q_dot * Tk;
    
    % update state
    R_s1 = R_so * Rot('z', q(1));
    R_s2 = R_s1 * Rot('x', -pi/2) * Trans('x', 0.160) * Rot('z', -pi/2+q(2));
    R_s3 = R_s2 * Trans('x', 0.980) * Rot('z', q(3));
    R_s4 = R_s3 * Rot('x', pi/2) * Trans('x', 0.150) * Rot('z', q(4)) * Trans('z', -0.860);
    R_s5 = R_s4 * Rot('x', -pi/2) * Rot('z', q(5));
    R_s6 = R_s5 * Rot('x', pi/2) * Rot('z', q(6));
    R_se = R_s6 * Rot('x', pi) * Trans('x', -0.120) * Trans('z', 0.303);

    distance = norm(R_se(1:3, 4) - p) - 3*sqrt(2)
    
% %     visualization
%     x = zeros(4, 1); y = zeros(4, 1); z = zeros(4, 1);
%     x(1) = R_so(1, 4); y(1) = R_so(2, 4); z(1) = R_so(3, 4);
%     x(2) = R_s2(1, 4); y(2) = R_s2(2, 4); z(2) = R_s2(3, 4);
%     x(3) = R_s3(1, 4); y(3) = R_s3(2, 4); z(3) = R_s3(3, 4);
%     x(4) = R_s4(1, 4); y(4) = R_s4(2, 4); z(4) = R_s4(3, 4);
%     x(5) = R_s5(1, 4); y(5) = R_s5(2, 4); z(5) = R_s5(3, 4);
%     x(6) = R_s6(1, 4); y(6) = R_s6(2, 4); z(6) = R_s6(3, 4);
%     x(7) = R_se(1, 4); y(7) = R_se(2, 4); z(7) = R_se(3, 4);
% 
%     figure(1);
%     hold on;
%     grid on;
%     plot3(x, y, z);
%     [x_s, y_s, z_s] = sphere;
%     r = 0.1;
%     surf(x_s*r+p(1), y_s*r+p(2), z_s*r+p(3))
%     axis equal
%     xlabel('x(t)')
%     ylabel('y(t)')
%     zlabel('z(t)')
%     view(10, 10);
end
toc

%% 2
x = zeros(4, 1); y = zeros(4, 1); z = zeros(4, 1);
x(1) = R_so(1, 4); y(1) = R_so(2, 4); z(1) = R_so(3, 4);
x(2) = R_s2(1, 4); y(2) = R_s2(2, 4); z(2) = R_s2(3, 4);
x(3) = R_s3(1, 4); y(3) = R_s3(2, 4); z(3) = R_s3(3, 4);
x(4) = R_se(1, 4); y(4) = R_se(2, 4); z(4) = R_se(3, 4);
figure(1);
hold on;
grid on;
plot3(x, y, z);
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
q = [0; 0; 0];
R_so = Trans('x', 3) * Trans('y', 3) * Rot('z', pi*5/4) * Trans('z', 0.520);
R_s1 = R_so * Rot('z', q(1));
R_s2 = R_s1 * Rot('x', -pi/2) * Trans('x', 0.160) * Rot('z', -pi/2+q(2));
R_s3 = R_s2 * Trans('x', 0.980) * Rot('z', q(3));
R_s4 = R_s3 * Rot('x', pi/2) * Trans('x', 0.150) * Rot('z', q(4)) * Trans('z', -0.860);
R_s5 = R_s4 * Rot('x', -pi/2) * Rot('z', q(5));
R_s6 = R_s5 * Rot('x', pi/2) * Rot('z', q(6));
R_se = R_s3 * Rot('x', pi) * Trans('x', -0.120) * Trans('z', 0.303);

distance = norm(R_se(1:3, 4) - p) - 3 * sqrt(2);

J = zeros(3, 6);
% weight = diag([25, 5, 1]);

w = 0.01;
[x_s, y_s, z_s] = sphere;
r = 0.1;
gain = 50;

elapsedTime = 0.0;
tic
% while (elapsedTime < 15)
while (true)
    % jacobian
    elapsedTime = toc;
    p(1) = 0.5 * cos(w * elapsedTime);
    p(2) = 0.5 * sin(w * elapsedTime);
    p_dot(1) = -0.5 * w * sin(w * elapsedTime);
    p_dot(2) = 0.5 * w * cos(w * elapsedTime);
    p_dot(3) = 0;
    
    J(:, 1) = cross(R_s1(1:3, 3), R_se(1:3, 4) - R_s1(1:3, 1));
    J(:, 2) = cross(R_s2(1:3, 3), R_se(1:3, 4) - R_s2(1:3, 4));
    J(:, 3) = cross(R_s3(1:3, 3), R_se(1:3, 4) - R_s3(1:3, 4));
    J(:, 4) = cross(R_s4(1:3, 3), R_se(1:3, 4) - R_s4(1:3, 4));
    J(:, 5) = cross(R_s5(1:3, 3), R_se(1:3, 4) - R_s5(1:3, 4));
    J(:, 6) = cross(R_s6(1:3, 3), R_se(1:3, 4) - R_s6(1:3, 4));
    J_reduced = [p' - R_se(1:3, 4)'] * J;
    
    % control (Task1 - P control)
    q_dot = J_reduced_pinv * (dot(p - R_se(1:3, 4), p_dot) + gain * distance);
    
    q = q + q_dot * Tk;
    
    % update state
    R_s1 = R_so * Rot('z', q(1));
    R_s2 = R_s1 * Rot('x', pi/2) * Trans('x', 0.160) * Rot('z', pi/2+q(2));
    R_s3 = R_s2 * Trans('x', 0.980) * Rot('z', q(3));
    R_se = R_s3 * Rot('x', pi/2) * Trans('x', 0.150) * Trans('z', 0.860);
    distance = norm(R_se(1:3, 4) - p) - 3*sqrt(2)
    
    % animation
    x = zeros(4, 1); y = zeros(4, 1); z = zeros(4, 1);
    x(1) = R_so(1, 4); y(1) = R_so(2, 4); z(1) = R_so(3, 4);
    x(2) = R_s2(1, 4); y(2) = R_s2(2, 4); z(2) = R_s2(3, 4);
    x(3) = R_s3(1, 4); y(3) = R_s3(2, 4); z(3) = R_s3(3, 4);
    x(4) = R_se(1, 4); y(4) = R_se(2, 4); z(4) = R_se(3, 4);
    figure(1);
    hold on;
    grid on;
    plot3(x, y, z);
    surf(x_s*r+p(1), y_s*r+p(2), z_s*r+p(3))
    axis equal
    xlabel('x(t)')
    ylabel('y(t)')
    zlabel('z(t)')
    view(10, 70);
end
toc

%% 4