function [R_se, J, x] = getJacobian2(q)
    R_so = Trans('x', 3) * Trans('y', 3) * Rot('z', pi*5/4) * Trans('z', 0.520);
    R_s1 = R_so * Rot('z', q(1));
    R_s2 = R_s1 * Rot('x', -pi/2) * Trans('x', 0.160) * Rot('z', -pi/2+q(2));
    R_s3 = R_s2 * Trans('x', 0.980) * Rot('z', q(3));
    R_s4 = R_s3 * Rot('x', pi/2) * Trans('x', 0.150) * Rot('z', q(4)) * Trans('z', -0.860);
    R_s5 = R_s4 * Rot('x', -pi/2) * Rot('z', q(5));
    R_s6 = R_s5 * Rot('x', pi/2) * Rot('z', q(6));
    R_se = R_s6 * Rot('x', pi) * Trans('x', -0.120) * Trans('z', 0.303 + 3*sqrt(2));
    
    J(1:3, 1) = cross(R_s1(1:3, 3), R_se(1:3, 4) - R_s1(1:3, 1));
    J(1:3, 2) = cross(R_s2(1:3, 3), R_se(1:3, 4) - R_s2(1:3, 4));
    J(1:3, 3) = cross(R_s3(1:3, 3), R_se(1:3, 4) - R_s3(1:3, 4));
    J(1:3, 4) = cross(R_s4(1:3, 3), R_se(1:3, 4) - R_s4(1:3, 4));
    J(1:3, 5) = cross(R_s5(1:3, 3), R_se(1:3, 4) - R_s5(1:3, 4));
    J(1:3, 6) = cross(R_s6(1:3, 3), R_se(1:3, 4) - R_s6(1:3, 4));
    J(4:6, 1) = R_s1(1:3, 3);
    J(4:6, 2) = R_s2(1:3, 3);
    J(4:6, 3) = R_s3(1:3, 3);
    J(4:6, 4) = R_s4(1:3, 3);
    J(4:6, 5) = R_s5(1:3, 3);
    J(4:6, 6) = R_s6(1:3, 3);
    
    x = zeros(7, 3); y = zeros(7, 1); z = zeros(7, 1);
    x(1, :) = R_so(1:3, 4)';
    x(2, :) = R_s2(1:3, 4)';
    x(3, :) = R_s3(1:3, 4)';
    x(4, :) = R_s4(1:3, 4)';
    x(5, :) = R_s5(1:3, 4)';
    x(6, :) = R_s6(1:3, 4)';
    x(7, :) = R_se(1:3, 4)';

end