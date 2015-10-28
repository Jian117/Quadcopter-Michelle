close all

b = 0.0000538; %na
d = 0.0000011; %na
l = 0.32;  %a
m = 1.6;
IXX = 0.1296;
IYY = 0.1296;
IZZ = 0.2272;
hl = 17.64;

T =100;
lift_step = 10;

%error input 
% no error
% vec_error = [0;0;0;0];

% all error
% vec_error = [-10.25;7.74;-55.555;0];

% roll error
 vec_error = [-5;0;0;0];

% pitch error
% vec_error = [0;7.74;0;0];

% yaw error
% vec_error = [0;0;55.55;0];


temp1 = 1/4/b;
temp2 = 1/2/l/b;
temp3 = 1/4/d;

% inverse motion matrix
IMM = [temp1, 0, 0-temp2, 0-temp3;
    temp1, 0-temp2, 0, temp3;
    temp1, 0 , temp2, 0-temp3;
    temp1, temp2, 0, temp3];

% error input vector
error = [0.0;0.0;0.0;0.0];
al = 0;

% error data storage
e = [0,0,0,0;
    0,0,0,0];

f = zeros(2,4);
U_vecs = zeros(4,T);
M2_vecs = zeros(4,T);

for n = 1:1:T
    if (n < 40)
        if (mod(n,10) == 0)
            al = al + lift_step;
        end
    end
    
    % inject roll errors
    if (n == 60)
        error = vec_error;
    end
    
    
    e(1,1) = error(1);
    e(1,2) = error(2);
    e(1,3) = error(3);
    e(1,4) = error(4);
    
    % ROLL PITCH YAW PIDs
    f(1,1) = (1.687 * e(1,1) - 1.633 * e(2,1) + 0.4913*f(2,1));
    f(1,2) = (1.687 * e(1,2) - 1.633 * e(2,2) + 0.4913*f(2,2));
    f(1,3) = (1.687 * e(1,3) - 1.633 * e(2,3) + 0.4913*f(2,3));
    %f(1,3) = (12.19 * e(1,3) - 11.16 * e(2,3) + 0.04173*f(2,3));
    f(1,4) = al;
    
    U_vecs(1,n) = f(1,4);
    U_vecs(2,n) = f(1,1);
    U_vecs(3,n) = f(1,2);
    U_vecs(4,n) = f(1,3);
    
    % store prev data
    e(2,1) = e(1,1);
    e(2,2) = e(1,2);
    e(2,3) = e(1,3);
    e(2,4) = e(1,4);
    f(2,1) = f(1,1);
    f(2,2) = f(1,2);
    f(2,3) = f(1,3);
    f(2,4) = f(1,4);   
    
    M2_vecs(:,n) = IMM*(f(1,:)');
    % INV
end

figure;
subplot(4,1,1);
plot(1:1:T, U_vecs(1,:));
title('U1');
subplot(4,1,2);
plot(1:1:T, U_vecs(2,:));
title('U2');
subplot(4,1,3);
plot(1:1:T, U_vecs(3,:));
title('U3');
subplot(4,1,4);
plot(1:1:T, U_vecs(4,:));
title('U4');
figure(2);
subplot(4,1,1);
plot(1:1:T, M2_vecs(1,:));
title('omega1_square');
subplot(4,1,2);
plot(1:1:T, M2_vecs(2,:));
title('omega2_square');
subplot(4,1,3);
plot(1:1:T, M2_vecs(3,:));
title('omega3_square');
subplot(4,1,4);
plot(1:1:T, M2_vecs(4,:));
title('omega4_square');