function [mat] = Trans(dir, ang)
mat = zeros(4, 4);
if (dir == 'x')
    mat = [1, 0, 0, ang;
        0, 1, 0, 0
        0, 0, 1, 0;
        0, 0, 0, 1];
        
elseif (dir == 'y')
    mat = [1, 0, 0, 0;
        0, 1, 0, ang;
        0, 0, 1, 0;
        0, 0, 0, 1];
elseif (dir == 'z')
    mat = [1, 0, 0, 0;
        0, 1, 0, 0;
        0, 0, 1, ang;
        0, 0, 0, 1];
end
end
