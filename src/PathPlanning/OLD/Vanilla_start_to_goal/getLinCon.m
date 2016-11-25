function [A,b,Aeq,beq] = getLinCon(n,Pstart,Pend)
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
xend=Pend(1);
yend=Pend(2);
thetaend=Pend(3);

% Esuality constraints to respect pose 
Apos1=[diag([1,1]),zeros(2,2*n-2)];
bpos1=[x1;y1];
Aposend=[zeros(2,2*n-2),diag([1,1])];
bposend=[xend;yend];
Adir1=[tan(theta1),-1,-tan(theta1),1,zeros(1,2*n-4)];
bdir1=0;
Adirend=[zeros(1,2*n-4),-tan(thetaend),1,tan(thetaend),-1];
bdirend=0;

Aeq=[Apos1;Aposend;Adirend];
beq=[bpos1;bposend;bdirend];

% Inequality constraints to respect orientation 
% A1=[1,0,-1,0,zeros(1,2*n-4)];
% A2=[0,1,0,-1,zeros(1,2*n-4)];
% A3=[zeros(1,2*n-4),1,0,-1,0];
% A4=[zeros(1,2*n-4),0,1,0,-1];
% A=[A1;A2;A3;A4];
% b=zeros(4,1);
end

