%% Plot Cylinder
%% Plot Robot
% Robot Animation
% hold on;
for i = 1:size(q,2)
    IRB1.plot(q(:,i)', 'delay', 0.001, 'jaxes','base', 'jointdiam', 0.3 , 'tilesize', 0.5, ...
        'workspace', [-1.5, 8, -3, 3, -1, 3])
%     IRB2.plot(q(:,i)', 'delay', 0.001, 'jaxes','base', 'jointdiam', 0.3 , 'tilesize', 0.5)
    hold on;
    ugv_base = inv(T_s0) * [ugv_global; 1];
    plot_cyl('z', 3, [0 3], 'c', [3*sqrt(2) 0 0]);
    plot_cyl('z', 0.1, [-0.1, 0.1], 'r', ugv_base);
    hold off;
end

