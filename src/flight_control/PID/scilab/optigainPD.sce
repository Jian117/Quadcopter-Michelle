//optigain.sce
//
//   search for optimal PD gains 
//           EE447   13-Oct-2011

exec('stepperf.sce');
 
t = 0:dt:tmax;

start_time = getdate("s");

////////////////////////////////////////////////////////////
//
//      function which evaluates controller performance, PID version
//
function [ts, po, ss, cu, y] = costPID(plant,Kp,Ki,Kd)
    //define controller

    maxval = 10; // max allowable value of step response (cheap stability test)
    pctl = pp*(Kd*s^2 + Kp*s + Ki)/(s*(s+pp)); 
    ctl  = syslin('c',pctl);
    // H=1 feedback 
    sys = ctl*plant /. H;   // H is  feedback
    ctl_effort = ctl /. plant;  //  get actuator output effort
    //pause;
    y = csim(ones(t),t,sys);  // get closed loop 
    u = csim(ones(t), t, ctl_effort);
   // pause;
    if (max(y) > maxval | min(y) < -maxval) then 
        ts = 999; po = 999; 
        else
          ts = settletime(t,y);
          po = overshoot(t,y);
          ss = sse(t,y);
          cu = max(u);
      end;      
endfunction
  
 
////////////////////////////   Parameter Searching   /////////////////////
//

// set up several weight schemes for performance 

//   wts = weight on settling time
//   wos = weight on overshoot
//   wsse = weight on steady state error
//   wu  = weight on control effort

Ns = 5;  
wnames(1) = sprintf("Ts = %5.3f",tsd);
wts(1) = 1.0    ;  wos(1) = 0.0  ;   wsse(1) = 0.0;  wu(1) = 0.0;

wnames(2) = sprintf("Overshoot = %5.2f",pod);
wts(2) = 0.0    ;  wos(2) = 1.0  ;   wsse(2) = 0.0;  wu(2) = 0.0;

wnames(3) = "SSE";
wts(3) = 0.0    ;  wos(3) = 0.0  ;   wsse(3) = 1.0;  wu(3) = 0.0;

wnames(4) = "Control Effort";
wts(4) = 0.0    ;  wos(4) = 0.0  ;   wsse(4) = 0.0;  wu(4) = 1.0;

wnames(5) = "Balanced: OS and SSE";
wts(5) = 0.00  ;  wos(5) = 0.5 ;   wsse(5) = 0.5;  wu(5) = 0.0; 



for(i=1:Ns),          //  initialize search storage
    emin(i) = 99999999.99;
    Kpo(i) = -1;
    Kio(i) = -1;
    Kdo(i) = -1;
end;

scalef = sqrt(scale_range);

kmin  = K1/scalef ;  kmax = K1*scalef;
kimin = K2/scalef;   kimax = K2*scalef;
kdmin = K3/scalef;   kdmax = K3*scalef;


// Increments
dk  = (kmax-kmin)/nvals;
dki = (kimax-kimin)/nvals;
dkd = (kdmax-kdmin)/nvals;

iter = 1;
////////////////// Print progress bar
printf("|");
for i=1:nvals+1, 
    printf("-"); 
    end
printf("|\n ");
 
 
 Ki = 0.0;          // PD controllers only

///////////////////////////////  Main Loop ////////////////////////////////
for Kp = kmin:dk:kmax, 
    printf("."); 
        for Kd = kdmin:dkd:kdmax,
       //     printf("%4d: K: %3.1f %3.1f %3.1f  emin(1): %f ", iter, Kp, Ki, Kd, emin(1));
            iter = iter + 1;
            [ts1, po1, ss, cu, y] = costPID(plant, Kp, Ki, Kd);
      //      printf("%4d: K: %3.1f %3.1f %3.1f ", iter, Kp, Ki, Kd);
       // printf("%4d ",iter);
            if(ts1 > 900) then // printf("*");
            else  //  if the system was stable
                 // printf( " ts1: %f po1: %f   eTs: %f  ePO: %f ", ts1, po1, e1, e2);
                 e1 = abs(ts1-tsd)/tsd; e2 = abs(po1-pod)/pod; e3 = abs( y(length(y))-1.0); e4 = cu/cu_max;
                 //  compute each weighted performance
                 for i = 1:Ns, 
                    e = wts(i)*e1 + wos(i)*e2 + wsse(i)*e3 + wu(i)*e4;
                    if(e < emin(i)) then emin(i) = e; Kpo(i) = Kp; Kio(i) = Ki; Kdo(i) = Kd;
                        end
                 end
       //       printf(" em1: %f em2: %f em3: %f em4: %f", emin(1), emin(2), emin(3), emin(4));
            end;
       //    printf("\n");
       //     e = e1*e2;
       //     if(e < emin) then emin = e; Kpo = Kp; Kio = Ki; Kdo = Kd; end;

        end
 end

printf("Optimization search complete!\n\n");

/////////////////////////////////////////   Print Results 
for i=1:Ns, 
   printf("\n\n[%15s] Kp: %6.3f  Ki: %6.3f  Kd: %6.3f\n",wnames(i), Kpo(i),Kio(i),Kdo(i));
   [ts1,po1, ss, cu, ybest] = costPID(plant, Kpo(i), Kio(i), Kdo(i));
   scf(i);    plot(t,ybest);  title(wnames(i));
   printf("Settling Time: %5.2f  Overshoot: %4.1f percent  SSE: %6.3f  Ctl Effort: %5.2f\n", ts1, (po1-1)*100.0, ss , cu);
   if(Kpo(i) == kmin | Kpo(i) == kmax | Kio(i) == kimin | Kio(i) == kimax | Kdo(i) == kdmin | Kdo(i) == kdmax) then 
       printf("  Search boundary reached:  ");
       if  Kpo(i) == kmin  then printf("Kp min "); end
       if  Kpo(i) == kmax  then printf("Kp max "); end
//       if  Kio(i) == kimin then printf("Ki min "); end
//       if  Kio(i) == kimax then printf("Ki max "); end
       if  Kdo(i) == kdmin then printf("Kd min "); end
       if  Kdo(i) == kdmax then printf("Kd max "); end
       printf("\n");  
   end
 end
 
 end_time = getdate("s");
 
 printf("\nSearch Time:  %3.1f minutes.  N = %d \n", (end_time-start_time)/60, (nvals+1)^3);
 
 
