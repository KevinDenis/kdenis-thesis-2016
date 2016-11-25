function [A,b,Aeq,beq] = getLinCon(n,Pstart)
%GETCONSTRAINTS Summary of this function goes here
%   Detailed explanation goes here
A=[];
b=[];
Aeq=[];
beq=[];
% z=[x1 y1 x2 y2 ...]

% pose
x1=Pstart(1);
y1=Pstart(2);
theta1=Pstart(3);


% Equality constraints to respect pose 
Apos=[diag([1,1]),zeros(2,2*n-2)];
bpos=[x1;y1];

Adir=[tan(theta1),-1,-tan(theta1),1,zeros(1,2*n-4)];
bdir=0;


Aeq=[Apos;Adir];
beq=[bpos;bdir];

% Inequality constraints to respect orientation 
A1=[1,0,-1,0,zeros(1,2*n-4)];
A2=[0,1,0,-1,zeros(1,2*n-4)];
% A3=[zeros(1,2*n-4),1,0,-1,0];
% A4=[zeros(1,2*n-4),0,1,0,-1];
A=[A1;A2];
b=zeros(2,1);
end

