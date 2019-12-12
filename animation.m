%% Plot Robot
% Robot Animation
for i = 1:250:size(q,2)
%     work_handle_cyl = plot_cyl('z', 1.5, [0 3], 'c', [1.5*sqrt(2) 0 0]);
    IRB1.plot(q(:,i)', 'delay', 0.001, 'jaxes','base', 'jointdiam', 0.3 , 'tilesize', 0.5, ...
        'workspace', [-1.5, 8, -3, 3, -1, 3]);
    
    ugv_base = inv(T_s0) * [ugv_global_arr(:,i); 1];
    hold on;
    work_handle = plot_cyl('z', 0.1, [-0.1, 0.1], 'r', ugv_base);
    work_handle2 = plot_cyl('z', 1, [-1 1], 'c', [1.5*sqrt(2) 0 0]);
    hold off;
    
    delete(work_handle);
end

%%
q = zeros(7,1);
i=1;
IRB1.plot(q(:,i)', 'delay', 0.001, 'jaxes','base', 'jointdiam', 0.3 , 'tilesize', 0.5, ...
    'workspace', [-1.5, 8, -3, 3, -1, 3]);