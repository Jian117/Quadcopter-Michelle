//optivis.sce
//
//      Visualize the performance space and 
//   search for optimal PID gains 
//

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

wnames(5) = "Balanced";
wts(5) = 0.25  ;  wos(5) = 0.25 ;   wsse(5) = 0.25;  wu(5) = 0.25; 



for(i=1:Ns),          //  initialize search storage
    emin(i) = 99999999.99;
    Kpo(i) = -1;
    Kio(i) = -1;
    Kdo(i) = -1;
end;

//  initialize graphics storage

ip = 1; ii = 1; id = 1;

Ts_vals(ip,ii,id) = 0.0;       //  Ts as a fcn of Kp, Ki, Kd

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
 
ip = 1;
///////////////////////////////  Main Loop ////////////////////////////////
for Kp = kmin:dk:kmax, 
    printf(".");
    xa(ip) = Kp;   // collect axis values for plots
    ii = 1;
    for Ki = kimin:dki:kimax,
        ya(ii) = Ki;
        id = 1;
        for Kd = kdmin:dkd:kdmax,
            za(id) = Kd;
       //     printf("%4d: K: %3.1f %3.1f %3.1f  emin(1): %f ", iter, Kp, Ki, Kd, emin(1));
            iter = iter + 1;
            [ts1, po1, ss, cu, y] = costPID(plant, Kp, Ki, Kd);
                    
      //      printf("%4d: K: %3.1f %3.1f %3.1f ", iter, Kp, Ki, Kd);
       // printf("%4d ",iter);
            if(ts1 > 900) then // unstable 
                e = 999; // printf("*");  
            else  //  if the system was stable
                 // printf( " ts1: %f po1: %f   eTs: %f  ePO: %f ", ts1, po1, e1, e2);
                 e1 = abs(ts1-tsd)/tsd; e2 = abs(po1-pod)/pod; e3 = abs( y(length(y))-1.0); e4 = cu/cu_max;
                 //  compute each weighted performance
                 for i = 1:Ns, //  go through the different weighting schemes
                    e = wts(i)*e1 + wos(i)*e2 + wsse(i)*e3 + wu(i)*e4;
                    if(e < emin(i)) then emin(i) = e; Kpo(i) = Kp; Kio(i) = Ki; Kdo(i) = Kd;
                        end
                 end
                 
       //       printf(" em1: %f em2: %f em3: %f em4: %f", emin(1), emin(2), emin(3), emin(4));
           end;
        
            Ts_vals(ip,ii,id) = ts1; // store settle time for graphics
            Bal_vals(ip,ii,id) = e;  // store Balanced score for graphics
            
       //    printf("\n");
       //     e = e1*e2;
       //     if(e < emin) then emin = e; Kpo = Kp; Kio = Ki; Kdo = Kd; end;
        id = id + 1;
        end
    ii = ii + 1;
    end
ip = ip + 1;
end

printf("Optimization search complete!\n\n");

/////////////////////////////////////////   Print Results 
for i=1:Ns, 
   printf("\n\n[%15s] Kp: %6.3f  Ki: %6.3f  Kd: %6.3f\n",wnames(i), Kpo(i),Kio(i),Kdo(i));
   [ts1,po1, ss, cu, ybest] = costPID(plant, Kpo(i), Kio(i), Kdo(i));
  // scf(i);    plot(t,ybest);  title(wnames(i));
   printf("Settling Time: %5.2f  Overshoot: %4.1f percent  SSE: %6.3f  Ctl Effort: %5.2f\n", ts1, (po1-1)*100.0, ss , cu);
   if(Kpo(i) == kmin | Kpo(i) == kmax | Kio(i) == kimin | Kio(i) == kimax | Kdo(i) == kdmin | Kdo(i) == kdmax) then 
       printf("  Search boundary reached:  ");
       if  Kpo(i) == kmin  then printf("Kp min "); end
       if  Kpo(i) == kmax  then printf("Kp max "); end
       if  Kio(i) == kimin then printf("Ki min "); end
       if  Kio(i) == kimax then printf("Ki max "); end
       if  Kdo(i) == kdmin then printf("Kd min "); end
       if  Kdo(i) == kdmax then printf("Kd max "); end
       printf("\n");  
   end
 end
 
 end_time = getdate("s");
 
 printf("\nSearch Time:  %3.1f minutes.  N = %d \n", (end_time-start_time)/60, (nvals+1)^3);
 
 
 

 
// xa = kmin  + dk*(0:nvals); 
// ya = kimin + dki*(0:nvals); 
// za = kdmin + dkd*(0:nvals);

if td_plots(TS) then
//
/////////////////////////////////////////  3d Ts plots
//

 for i=1:nvals+1
     for j=1:nvals+1;
         z(i,j) = Ts_vals(i,j,int(nvals/2));
     end
 end
 scf(8);
plot3d1(xa,ya,z);
title("Settling Time vs. Kp(x), Ki(y) for middle value of Kd");
 
 
scf(9);
plot3d1(xa,za,z);
title("Settling Time vs. Kp(x), Kd(y) for middle value of Ki");
end



if maps(TS) then
//   3D plots of Ts vs Kp, Ki, Kd
//
////////////////////////////////    Ts color plots
// 
 
// Set arbitrary fixed color scale for Ts
Ts_max = 8;
Ts_min = 0; 


scf(12);///////////////////////////////   scf(12)

 for i=1:nvals+1
     for j=1:nvals+1;
         z(i,j) = Ts_vals(i,int(nvals/2),j);  // plot of Kp vs Kd
     end
 end
xset("colormap",jetcolormap(64));
colorbar(Ts_min, Ts_max);
da=gca() // get the handle on axes model to view and edit the fields
da.title.text="Color key";
da.x_label.text="";
da.y_label.text=""; 
Sgrayplot(xa,za,z,zminmax=[Ts_min, Ts_max]);
da=gca() // get the handle on axes model to view and edit the fields
// title by default
da.title.text="Ts for Ki = Initial"
da.x_label.text="Kp";
da.y_label.text="Kd"; 
 
 
 
scf(13);/////////////////////////////////  scf(13)
for i=1:nvals+1
     for j=1:nvals+1;
         z(i,j) = Ts_vals(i,j, int(nvals/2));  // plot of Kp vs Ki
     end
end

xset("colormap",jetcolormap(64));
colorbar(Ts_min, Ts_max);
da=gca() // get the handle on axes model to view and edit the fields
da.title.text="Color key";
da.x_label.text="";
da.y_label.text=""; 
Sgrayplot(xa,ya,z,zminmax=[Ts_min, Ts_max]); 
da=gca(); // get the handle on axes model
da.title.text="Ts for Kd = Initial"
da.x_label.text="Kp";
da.y_label.text="Ki"; 
 
end

//
//////////////////////////////////  Balanced 3d plots
//

if td_plots(BAL) then

for i=1:nvals+1
    for j=1:nvals+1;
        z(i,j) = Bal_vals(i,j,int(nvals/2));
    end
end
scf(15);
plot3d1(xa,ya,z);
title("Balanced Score vs. Kp(x), Ki(y) for middle value of Kd");
 
 
scf(16);

for i=1:nvals+1
    for j=1:nvals+1;
        z(i,j) = Bal_vals(i,int(nvals/2),j);
    end
end
endplot3d1(xa,za,z);
title("Balanced Score vs. Kp(x), Kd(y) for middle value of Ki");

end 

//
////////////////////////////////    Balanced color plots
// 
 
 
Bmax = 1.0;
Bmin = 0;

if maps(BAL) then
scf(14);///////////////////////////////////   scf(14)

 for i=1:nvals+1
     for j=1:nvals+1;
         z(i,j) = Bal_vals(i,int(nvals/2),j);  // plot of Kp vs Kd
     end
 end
xset("colormap",jetcolormap(64)); 
colorbar(Bmin, Bmax); 
Sgrayplot(xa,za,z, zminmax=[Bmin, Bmax]);
da=gca() // get the handle on axes model to view and edit the fields
// title by default
da.title.text="Balanced Score  for Ki = Initial"
da.x_label.text="Kp";
da.y_label.text="Kd"; 
 

scf(15);////////////////////////////////////   scf(15)
for i=1:nvals+1
     for j=1:nvals+1;
         z(i,j) = Bal_vals(i,j, int(nvals/2));  // plot of Kp vs Ki
     end
end

xset("colormap",jetcolormap(64)); 
colorbar(Bmin, Bmax); 
Sgrayplot(xa,ya,z,zminmax=[Bmin, Bmax]);
da=gca(); // get the handle on axes model
da.title.text="Balanced Score for Kd = Initial"
da.x_label.text="Kp";
da.y_label.text="Ki"; 

end 
 
 