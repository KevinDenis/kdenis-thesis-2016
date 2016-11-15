function [A,b,Aeq,beq] = getLinCon(n,Pstart,Pend)
%GETCONSTRAINTS Summary of this function goes here
%   Detailed explanation goes here
A=[];
b=[];
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
Adir1=[0,0,-tan(theta1),1,zeros(1,2*n-4)];
bdir1=-tan(theta1)*x1+y1;
Adirend=[zeros(1,2*n-4),tan(thetaend),-1,0,0];
bdirend=tan(thetaend)*xend-yend;
Aposend=[zeros(2,2*n-2),diag([1,1])];
bposend=[xend;yend];

Aeq=[Apos1;Aposend;Adir1;Adirend];
beq=[bpos1;bposend;bdir1;bdirend];

% % Inequality constraints to respect orientation 
% A=zeros(4,2*n);
% A(1,3)=-1;
% A(2,4)=-1;
% A(3,end-3)=1;
% A(4,end-2)=1;
% b=[-x1;-y1;xend;yend];

end

