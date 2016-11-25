function [COP] = getCOP(alpha,n,obs,Pstart,Pgoal)
%[COP] = getCOP(alpha,n)
%   Detailed explanation goes here
t=linspace(0,1,101)';

x1=Pstart(1); y1=Pstart(2); theta1=Pstart(3);
xg=Pgoal(1); yg=Pgoal(2); thetag=Pgoal(3);

z0=zeros(2*n,1);
z0(1:2:end)=linspace(x1,xg,n)';
% z0(2:2:end)=zeros(n,1);
z0(2:2:end)=linspace(y1,yg,n)';

z0(2:2:end)=[y1 y1 yg yg];

% z0([2 end])=[y1 yg];

lb = ones(2*n,1);
lb(1:2:end)=x1-1; 
lb(2:2:end)=y1-1;

ub = zeros(2*n,1);
ub(1:2:end)=xg+2; 
ub(2:2:end)=yg+2;

[A,b,Aeq,beq] = getLinCon(n,Pstart);

% options = optimoptions('Display', 'off');
% options = optimset('Display', 'off') ;
COP.options = [];
COP.solver = 'fmincon';
COP.objective = @(z) objectiveFunction(z(1:2:end),z(2:2:end),alpha,Pstart,Pgoal);
COP.x0 = z0;
COP.A=A;
COP.b=b;
COP.Aeq=Aeq;
COP.beq=beq;
COP.lb=lb;
COP.ub=ub;
COP.nonlcon = @(z) getNonLinCon(z(1:2:end),z(2:2:end),t,obs);
end

