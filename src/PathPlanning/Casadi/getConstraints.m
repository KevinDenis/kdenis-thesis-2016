function [A,b,Aeq,beq] = getConstraints(n)
%GETCONSTRAINTS Summary of this function goes here
%   Detailed explanation goes here

% z=[x1 y1 x2 y2 ...]

% pose
x1=0;
y1=0;
theta1=0;
xend=2;
yend=2;
thetaend=pi/2;

A1=[diag([1,1]),zeros(2,2*n-2)];
b1=[x1;y1];
A2=[0,0,-tan(theta1),1,zeros(1,2*n-4)];
b2=-tan(theta1)*x1+y1;
Apend=[zeros(1,2*n-4),-tan(thetaend),1,0,0];
bpend=-tan(thetaend)*xend+yend;
Aend=[zeros(2,2*n-2),diag([1,1])];
bend=[xend;yend];
Aeq=[A1;A2;Apend;Aend];
beq=[b1;b2;bpend;bend];

A=[zeros(2*n-4,2),diag([-ones(1,n-2),ones(1,n-2)]),zeros(2*n-4,2)];
b=[-x1;-y1;xend;yend];

A=[];
b=[];
end

