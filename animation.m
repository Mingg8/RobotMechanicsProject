%% Plot Robot
% Robot Animation
close all;
% [x1, y1, z1] = plot_cyl('z', 1, [-1 1], [1.5*sqrt(2) 0 0]);
[x1, y1, z1] = plot_cyl('z', 1, [-1 1], [3.0*sqrt(2) 0 0]);
surf(x1,y1,z1, 'FaceColor', 'c', 'EdgeColor', 'none')
work_handle2 = patch(x1', y1', z1', 'c');
for i = 1:10:size(q,2)
    IRB1.plot(q(:,i)', 'delay', 0.001, 'jaxes','base', 'jointdiam', 0.3 , 'tilesize', 0.5, ...
        'workspace', [-1.5, 8, -3, 3, -1, 3]);
    ugv_global_arr(:, i)
    ugv_base = inv(T_s0) * [ugv_global_arr(:,i); 1];
    hold on;
    [x, y, z] = plot_cyl('z', 0.1, [-0.1, 0.1], ugv_base);
    surf(x,y,z, 'FaceColor', 'r', 'EdgeColor', 'none')
    work_handle = patch(x', y', z', 'r');
    
    [T_0e, J, x] = getJacobian(q(:,i));
    
    [x_r, y_r, z_r] = plot_cyl('z', 0.02, [0, 3*sqrt(2)], [0; 0; 0]);
    for i = 1:2
        tmp = T_0e*[x_r(i,:); y_r(i,:); z_r(i,:); ones(1,size(x_r,2))];
        x_r(i, :) = tmp(1, :);
        y_r(i, :) = tmp(2, :);
        z_r(i, :) = tmp(3, :);
    end
    surf(x_r, y_r, z_r, 'FaceColor', 'b', 'EdgeColor', 'none')
    work_handle_rf = patch(x_r', y_r', z_r', 'r');
    
    
    hold off; 
    drawnow;
    
%     delete(work_handle);
end

%%
q = zeros(7,1);
i=1;
IRB1.plot(q(:,i)', 'delay', 0.001, 'jaxes','base', 'jointdiam', 0.3 , 'tilesize', 0.5, ...
    'workspace', [-1.5, 8, -3, 3, -1, 3]);