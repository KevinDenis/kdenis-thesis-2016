function [COP] = getCOP(alpha,n,r,obs)
%[COP] = getCOP(alpha,n)
%   Detailed explanation goes here
t=linspace(0,1,101)';

x1=0;
y1=0;
theta1=0;
Pstart=[x1,y1,theta1];

xend=2;
yend=2;
thetaend=pi/2;
Pend=[xend,yend,thetaend];

z0=zeros(2*n,1);
z0(1:2:end)=linspace(x1,xend,n);
z0(2:2:end)=zeros(n,1);z0(end)=yend;

lb = ones(2*n,1);
lb(1:2:end)=x1; 
lb(2:2:end)=y1;

ub = zeros(2*n,1);
ub(1:2:end)=xend; 
ub(2:2:end)=yend;

[A,b,Aeq,beq] = getLinCon(n,Pstart,Pend);

% options = optimoptions('Display', 'off');
% options = optimset('Display', 'off') ;
COP.options = [];
COP.solver = 'fmincon';
COP.objective = @(z) costFunction(z(1:2:end),z(2:2:end),alpha);
COP.x0 = z0;
COP.A=A;
COP.b=b;
COP.Aeq=Aeq;
COP.beq=beq;
COP.lb=lb;
COP.ub=ub;
COP.nonlcon = @(z) getNonLinCon(z(1:2:end),z(2:2:end),t,r,obs);
end

