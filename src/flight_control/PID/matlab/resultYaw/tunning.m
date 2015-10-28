% PID controller for the quadcopter
% YAW

clear all
clc

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

% choose pitch yaw or roll
I = IZZ;

% plant definition
P_deno  = sym2poly(I*s^2);
P_num = 1;
P = tf(P_num, P_deno);

% sensor def
H = 1;

Kp = 12.306;
Ki = 0;
Kd = 6.015;

% controller type thing
rho = 10;
rho_deno = sym2poly(rho + s);
P_num = 1;
that_thing = tf(rho, rho_deno);

C = pid(Kp, Ki, Kd);

T = feedback(C*P, H);

% closed loop reponse to the PID controller
%step(T)

%plant response
%step(P)

% open up the tuner
pidtool(P)
