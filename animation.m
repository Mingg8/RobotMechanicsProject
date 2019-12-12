%% Plot Robot
% Robot Animation
[x1, y1, z1] = plot_cyl('z', 1, [-1 1], [1.5*sqrt(2) 0 0]);
surf(x1,y1,z1, 'FaceColor', 'c', 'EdgeColor', 'none')
work_handle2 = patch(x1', y1', z1', 'c');
for i = 1:250:size(q,2)
%     work_handle_cyl = plot_cyl('z', 1.5, [0 3], 'c', [1.5*sqrt(2) 0 0]);
    IRB1.plot(q(:,i)', 'delay', 0.001, 'jaxes','base', 'jointdiam', 0.3 , 'tilesize', 0.5, ...
        'workspace', [-1.5, 8, -3, 3, -1, 3]);
    
    ugv_base = inv(T_s0) * [ugv_global_arr(:,i); 1];
    hold on;
    [x, y, z] = plot_cyl('z', 0.1, [-0.1, 0.1], ugv_base);
    surf(x,y,z, 'FaceColor', 'r', 'EdgeColor', 'none')
    work_handle = patch(x', y', z', 'r');
    hold off;
    drawnow;
    
    delete(work_handle);
end

%%
q = zeros(7,1);
i=1;
IRB1.plot(q(:,i)', 'delay', 0.001, 'jaxes','base', 'jointdiam', 0.3 , 'tilesize', 0.5, ...
    'workspace', [-1.5, 8, -3, 3, -1, 3]);