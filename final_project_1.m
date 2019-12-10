%% #1
clc;
qi = [0; 0; 0; 0; 0; 0; 0];

ugv_global = [0; 0; 0];
ugv_base = inv(T_s0) * [ugv_global; 1];
[R_se, J, tmp] = getJacobian(qi);
pos = R_se(1:3, 4) + R_se(1:3, 1:3) * [0; 0; 3*sqrt(2)];
distance = norm(pos - ugv_base(1:3));
dist_arr = [];

gain = 50;

q = [];
while (abs(distance) > 1e-4)
    % control
    mat = [0, -3*sqrt(2), 0;
        3*sqrt(2), 0, 0;
        0, 0, 0];
    J1 = [eye(3) -R_se(1:3, 1:3)* mat *R_se(1:3, 1:3)'] *J;
    i = i + 1;
    if (det(J1 * J1') < 1e-1) 
        q_dot = -J1'*inv(J1*J1'+0.01 * eye(3)) * gain * [(pos-ugv_base(1:3))];
    else
        q_dot = -pinv(J1) * gain * [(pos - ugv_base(1:3))];
    end
    
    qi = qi + [q_dot * Tk; 0];
    q = [q qi];
    i = i+1;
    
    % update
    [R_se, J, tmp] = getJacobian(qi);
    pos = R_se(1:3, 4) + R_se(1:3, 1:3) * [0; 0; 3*sqrt(2)];
    distance = norm(pos - ugv_base(1:3));
    dist_arr = [dist_arr; distance];
end