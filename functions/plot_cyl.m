function [x, y, z] = plot_cyl(ax, r, extent,offset)
    n = 20;
    theta = (0:n)/n*2*pi;
    % default value for offset
    if isempty(offset)
        offset = [0 0 0];
    end

    r = [r;r];
    n = length(theta)-1;

    switch ax
        case 'x'
            y = r * cos(theta);
            z = r * sin(theta);
            x = extent(:) * ones(1,n+1);
        case 'y'
            x = r * cos(theta);
            z = r * sin(theta);
            y = extent(:) * ones(1,n+1);
        case 'z'
            y = r * cos(theta);
            x = r * sin(theta);
            z = extent(:) * ones(1,n+1);
    end

    x = x + offset(1);
    y = y + offset(2);
    z = z + offset(3);

%     % walls of the shape
%     surf(x,y,z, 'FaceColor', 'c', 'EdgeColor', 'none')
% 
%     % put the ends on
%     work_handle = patch(x', y', z', color, 'EdgeColor', 'none');
    alpha(0.3);
end

