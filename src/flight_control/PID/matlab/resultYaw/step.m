% yaw
syms s
%constants
b = 0.0000538; %na
d = 0.0000011; %na
l = 0.32;  %a
m = 1.6;
IXX = 0.1296;
IYY = 0.1296;
IZZ = 0.2272;
hl = 17.64;
al = 20;
Kd = 0.095491;
Kp = 0.39099;
Ki = 0;

sampling_rate = 0.0625; % sampling period
k = 4;

I = IZZ;

% PD controller
h = tf([Kd/k+Kd, Kp], [Kd/Kp/k, 1]);

hd = c2d(h, sampling_rate, 'tustin');

% plant definition
P_deno  = sym2poly(I*s^2);
P_num = 1;
P = tf(P_num, P_deno);

oc = P * h;

H = 1;

T = feedback(oc, H);
Tcf = feedback(h, H*P);

subplot(4,1,1);
% controller response
step(h, '-', hd, '--');
title('controller response')
subplot(4,1,2);
% open loop response
step(oc, '-');
title('open loop response')
subplot(4,1,3);
% closed loop response
step(T, '-')
title('close loop response')
subplot(4,1,4);
% control effort
step(Tcf, '-');
title('controller effort')
