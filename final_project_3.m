%% #3
p = [0.5; 0.0; 0];
qi = [0; 0; 0; 0; 0; 0; 0];

[R_se, J, tmp] = getJacobian_spatial(qi);
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
damping = 0.01;

elapsedTime = 0.0;

tic
q = [];
while (elapsedTime < 3.0)
    elapsedTime = toc;
    p(1) = 0.5 * cos(w * elapsedTime);
    p(2) = 0.5 * sin(w * elapsedTime);

    % control
    mat = [0, -3*sqrt(2), 0;
        3*sqrt(2), 0, 0;
        0, 0, 0];
    J1 = [eye(3) -R_se(1:3, 1:3)* mat *R_se(1:3, 1:3)'] *J;
    P1 = eye(6) - pinv(J1) * J1;
    J2 = [zeros(3) eye(3)] * J; % 3 by 6
%     P2 = P1 - pinv(J2 * P1) * J2 * P1;
%     J3 = [1 0 0 0 0 0 ];
%     J3 * P2
    
    r1_dot = -gain * [(pos - p)];
    r2_dot = zeros(3, 1);
    
    if (det(J1 * J1') < 1e-1)
        q1_dot = J1' * inv(J1 * J1' + damping * eye(3)) * r1_dot;
    else
        q1_dot = pinv(J1) * r1_dot;
    end
    if (det(J2 * P1 * P1' * J2') < 1e-1)
        q2_dot = q1_dot + (J2 * P1)' * inv(J2 * P1 * P1' * J2' + eye(3) * damping) ...
            * (r2_dot - J2 * q1_dot);
    else
        q2_dot = q1_dot + pinv(J2 * P1) * (r2_dot - J2 * q1_dot);
    end
    
    qi = qi + [q2_dot * Tk; 0];
    q_sens = q_sens + (rand(1)-0.5) / 180.0 * pi;
    q = [q qi];
    
    % update
    [R_se, J, tmp] = getJacobian_spatial(qi);
    pos = R_se(1:3, 4) + R_se(1:3, 1:3) * [0; 0; 3*sqrt(2)];
    
    % for debugging
    distance = norm(pos - p);
end
toc
