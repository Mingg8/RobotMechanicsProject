function [mat] = Rot(dir, ang)
mat = zeros(4, 4);
if (dir == 'x')
    mat = [1, 0, 0, 0;
        0, cos(ang), -sin(ang), 0;
        0, sin(ang), cos(ang), 0;
        0, 0, 0, 1];
        
elseif (dir == 'y')
    mat = [cos(ang), 0, sin(ang), 0;
        0, 1, 0, 0;
        -sin(ang), 0, cos(ang), 0;
        0, 0, 0, 1];
elseif (dir == 'z')
    mat = [cos(ang), -sin(ang), 0, 0;
        sin(ang), cos(ang), 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1];
end
end

