clc; clear; close all;
startup_rvc;

%% Setup
Tk = 0.001;
alpha = [0; -pi/2; 0;     pi/2;  -pi/2; pi/2; pi];
a =     [0; 0.160; 0.980; 0.150; 0;    0;      -0.120];  
offset =[0; -pi/2; 0;     0;     0;   0;       0];
d =     [0;   0;    0;   -0.860; 0;    0;       0.303]; 

i = 1;
L(i) = Link('d', d(i), 'a', a(i), 'alpha', alpha(i), 'offset', offset(i), 'modified'); 
for i = 2:7
    L(i) = Link('d', d(i), 'a', a(i), 'alpha', alpha(i), 'offset', offset(i), 'modified'); 
end

IRB1 = SerialLink(L);
IRB2 = SerialLink(L);
IRB3 = SerialLink(L);
IRB4 = SerialLink(L);


T_s0 = Trans('x', 3) * Trans('y', 3) * Rot('z', pi*5/4) * Trans('z', 0.520);
T_6e = Rot('x', pi) * Trans('x', -0.120) * Trans('z', 0.303);

%%
% final_project_1
% final_project_3
final_project_4

%%
animation

%% Test
clc;
q = [0; 0; 0; 0; 0; 0; 0];
animation