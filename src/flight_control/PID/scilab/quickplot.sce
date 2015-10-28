// Plot response of a PID controller and plant

xdel(winsid());   // close all graphics windows which might be open
exec('stepperf.sce');
t = 0:(0.1*dt):tmax;

Kp = input("KP: ");
Ki = input("Ki: ");
Kd = input("Kd: ");


//    Set up the closed loop system

    pctl = pp*(Kd*s^2 + Kp*s + Ki)/(s*(s+pp)); 
    ctl  = syslin('c',pctl);
    // H=1 feedback 
    sys = ctl*plant /. H;   // H is  feedback
    
    
y = csim(ones(t), t, sys);

scf(8);

plot(t,y)
title("Step response");


[ts1, po1, ss, cu, y] = costPID(plant, Kp, Ki, Kd);

printf("Settling Time: %5.2f  Overshoot: %4.1f percent  SSE: %6.3f  Ctl Effort: %5.2f\n", ts1, (po1-1)*100.0, ss , cu);

scf(9);

loopgain = ctl*plant;

printf("Gain Margin: %5.1f dB,  Phase Margin: %5.1f deg\n", g_margin(loopgain), p_margin(loopgain));

show_margins(loopgain);    // gain and phase margins of loop gain. 

printf("Factored Controller: ")
rlist=roots(s^2+Kp/Kd*s+Ki/Kd)';
a=rlist(1); b=rlist(2);
if (abs(imag(a)) > 10^(-5) ) then {
  printf("%12.9f x (s+%6.4f+j%6.4f)(s+%6.4f+j%6.4f)", Kd, -real(a), -imag(a), -real(b), -imag(b)) ; };  
else 
      { printf("%12.9f x (s+%6.4f)(s+%6.4f)",  Kd,  -a, -b); }

