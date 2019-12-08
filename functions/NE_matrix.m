function [M,Mdot,C,Grav] = NE_matrix(q, dq) %ML= [MX, MY, MZ]

m1=1;
MX1=0.5;
MY1=0.5;
MZ1=0.5;
J_joint_x1=0.5;
J_joint_y1=0.5;
J_joint_z1=0.0067;
J_joint_xy1=0.5;
J_joint_xz1=0.5;
J_joint_yz1=0.5;

m2=1;
MX2=-0.0057;
MY2=-1.1325;
MZ2=0.5;
J_joint_x2=-0.0155;
J_joint_y2=0.0223;
J_joint_z2=0.0231;
J_joint_xy2=-0.0053;
J_joint_xz2=0.0284;
J_joint_yz2=-0.0035;

m3=0.3586;
MX3=0.4993;
MY3=0.0119;
MZ3=1.1373;
J_joint_x3=-0.0152;
J_joint_y3=0.0228;
J_joint_z3=0.0373;
J_joint_xy3=0.0007;
J_joint_xz3=-0.0107;
J_joint_yz3=-0.0048;

m4=0.4002;
MX4=-0.3345;
MY4=0.5092;
MZ4=-0.0118;
J_joint_x4=-0.0293;
J_joint_y4=0.0513;
J_joint_z4=-0.0016;
J_joint_xy4=0.0506;
J_joint_xz4=0.0047;
J_joint_yz4=-0.0027;

m5=0.6207;
MX5=-0.01;
MY5=0.0385;
MZ5=0.4897;
J_joint_x5=-0.0046;
J_joint_y5=-0.0263;
J_joint_z5=0.0076;
J_joint_xy5=-0.0037;
J_joint_xz5=-0.0061;
J_joint_yz5=0.0077;

m6=0.6207;
MX6=0.1102;
MY6=-0.0338;
MZ6=-0.0385;
J_joint_x6=-0.0025;
J_joint_y6=0.0056;
J_joint_z6=0.0137;
J_joint_xy6=0.0024;
J_joint_xz6=0;
J_joint_yz6=0.0008;

J_joint = zeros(3,3,7);
J_joint(:,:,1) = [J_joint_x1, J_joint_xy1, J_joint_xz1;
    J_joint_xy1, J_joint_y1, J_joint_yz1;
    J_joint_xz1, J_joint_yz1, J_joint_z1];
J_joint(:,:,2) = [J_joint_x2, J_joint_xy2, J_joint_xz2;
    J_joint_xy2, J_joint_y2, J_joint_yz2;
    J_joint_xz2, J_joint_yz2, J_joint_z2];
J_joint(:,:,3) = [J_joint_x3, J_joint_xy3, J_joint_xz3;
    J_joint_xy3, J_joint_y3, J_joint_yz3;
    J_joint_xz3, J_joint_yz3, J_joint_z3];
J_joint(:,:,4) = [J_joint_x4, J_joint_xy4, J_joint_xz4;
    J_joint_xy4, J_joint_y4, J_joint_yz4;
    J_joint_xz4, J_joint_yz4, J_joint_z4];
J_joint(:,:,5) = [J_joint_x5, J_joint_xy5, J_joint_xz5;
    J_joint_xy5, J_joint_y5, J_joint_yz5;
    J_joint_xz5, J_joint_yz5, J_joint_z5];
J_joint(:,:,6) = [J_joint_x6, J_joint_xy6, J_joint_xz6;
    J_joint_xy6, J_joint_y6, J_joint_yz6;
    J_joint_xz6, J_joint_yz6, J_joint_z6];
J_joint(:,:,7) = [J_joint_x7, J_joint_xy7, J_joint_xz7;
    J_joint_xy7, J_joint_y7, J_joint_yz7;
    J_joint_xz7, J_joint_yz7, J_joint_z7];

ML = [MX1, MX2, MX3, MX4, MX5, MX6, MX7;
    MY1, MY2, MY3, MY4, MY5, MY6, MY7;
    MZ1, MZ2, MZ3, MZ4, MZ5, MZ6, MZ7];

m = [m1,m2,m3,m4,m5,m6,m7];

g = [0;0;-9.8];
%% A mat
Arot = [0;0;1;0;0;0];
Alin = [0;0;0;0;0;1];
A = zeros(6*6,6);
for i = 1:6
    A(6*(i-1)+1:6*i,i) = Arot;
end

%% SO(3) & Frame Position Vector
alpha = [0; -pi/2; 0;     pi/2;  -pi/2; pi/2];
a =     [0; 0.160; 0.980; 0.150; 0;    0];  
theta = [0; -pi/2; 0;     0;     0;    0];
d =     [0;   0;    0;   -0.860; 0;     0];

R = zeros(3,3*6);
p_body_pre = zeros(3,6); % e.g. p_01_0
p_body_aft = zeros(3,6); % e.g. p_10_1

for i = 1:6
    T_tmp = mod_DH_T(a(i),d(i),alpha(i),q(i) + theta(i)); %R_i_i+1
    R(1:3,3*(i-1)+1:3*i) = T_tmp(1:3,1:3);
    p_body_pre(:,i) =T_tmp(1:3,4);
    % p_body_pre(i) = p_i_i+1
end

for i = 1:6
    p_body_aft(:,i) = -transpose(R(1:3,3*(i-1)+1:3*i))*p_body_pre(:,i);
    % R_i+1_i * p_body_pre
    % (position of i+1 frame) (respect to i frame) (in i+1 coordinate)
    % p_(i+1)_i^(i+1)
end

%% SE(3)
T = zeros(4,4,6);
    for i = 1:6 
        T(:,:,i) = [transpose(R(1:3,3*(i-1)+1:3*i)), p_body_aft(:,i);0 0 0 1];
        % T_i+1_i
    end

%% G mat
Gi = sym(zeros(7*6,7*6));
count = 1;
for i=1:9
        Gi(6*(i-1)+1:6*i,6*(i-1)+1:6*i) = [J_joint(:,:,count), skew(ML(:,count));-skew(ML(:,count)), m(count)*eye(3)];
        count = count + 1;
end



%% NE
    % W
    W1 = zeros(6*7,6*7);
    for i = 1:5
        W1(6*i+1:6*(i+1),6*(i-1)+1:6*i) = Ad(T(:,:,i+1));
    end


    % L
    LL = zeros(6*6,6*6);
    for i = 1:6
        for j = i:6
            T_temp = eye(4);
            if j == i
                LL(6*(j-1)+1:6*j,6*(i-1)+1:6*i) = eye(6);
            else
                for k = i+1:j
                    T_temp = T(:,:,k)*T_temp;
                end
                LL(6*(j-1)+1:6*j,6*(i-1)+1:6*i) = Ad(T_temp);
            end
        end
    end

    %% Forward Iteration
    dV0 = [0;0;0;-g];

    V = zeros(6,1,6);
    V(:,:,1) = Arot*dq(1);
    for i = 2:6
            V(:,:,i) = Ad(T(:,:,i))*V(:,:,i-1) + Arot*dq(i);
    end

    ad_V = zeros(6*6,6*6);

    for i = 1:6
        ad_V(6*(i-1)+1:6*i,6*(i-1)+1:6*i) = adj(V(:,:,i));
    end
    LLdot = LL*ad_V-ad_V*LL;
    dV_base = [Ad(T(:,:,1))*dV0;zeros(30,1)];
    %% parameter
    M = transpose(A)*transpose(LL)*Gi*LL*A;
    Mdot = transpose(A)*transpose(LLdot)*Gi*LL*A + transpose(A)*transpose(LL)*Gi*LLdot*A;
    C = transpose(A)*transpose(LL)*(Gi*LL*ad_V-transpose(ad_V)*Gi*LL)*A;

    Grav = transpose(A)*transpose(LL)*Gi*LL*dV_base;
end