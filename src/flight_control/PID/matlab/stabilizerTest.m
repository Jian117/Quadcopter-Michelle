close all;
clear all;
clc

e = 0;
e1 = 0;
e2 = 0;
f1 = 0;
f2 = 0;

for i = 1:5;
    e = 5 - i;
    
    f = stabilizer(e, e1, e2, f2);
    
    e2 = e1;
    e1 = e;
    f2 = f1;
    f1 = f;
    
    results(i) = f;
end
