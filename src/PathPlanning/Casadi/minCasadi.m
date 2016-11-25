function [Amin] = minCasadi(A)
%MINCASADI Summary of this function goes here
%   Detailed explanation goes here
clear
close all
clc


P=[0,0];
obs=[1.0 0.5; 
     1.5 0.5;
     1.5 0.0;
     1.0 0.0];
 

k = zeros(size(P,1),1);
d = zeros(size(P,1),1);
for i = 1:size(P,1) 
    yi = repmat(P(i,:),size(obs,1),1);
    sumation=sum((obs-yi).^2,2)
    [d(i),k(i)] = min(sumation);
end

import casadi.*
x= MX.sym('x',1,1); x(1,1)=0;
y= MX.sym('y',1,1); y(1,1)=0;
P=[x y]

import casadi.*

x = MX.sym('x',10);

m = x(1);
for i=2:10
    m = min(m, x(i));
end

y = find(x == m);

f = Function('f',{x},{m,y});

[a,b] = f([2 3 1 4 -1 5 6 7 8 9]);
b = b+1;

disp('minimal element');
a
disp('index');
b

end

