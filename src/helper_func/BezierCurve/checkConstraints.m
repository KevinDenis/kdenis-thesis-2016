function [satisCons] = checkConstraints(x,y,initCOP)
%CHECKCONSTRAINTS Summary of this function goes here
%   Detailed explanation goes here

t=initCOP.t;
P0=initCOP.P0;
P1=initCOP.P1;
kappa_max=initCOP.kappa_max;

x0=P0(1); y0=P0(2); th0=P0(3);
x1=P1(1); y1=P1(2); th1=P1(3);

[~,~,~,kappa]=BezierCurve(x,y,t);

cons=boolean(zeros(1,1));
tol=1e-2;

cons(1)=IsNear(x0,x(1),tol);
cons(2)=IsNear(y(1),y0,tol);
cons(3)=IsNear(x(end),x1,tol);
cons(4)=IsNear(y(end),y1,tol);

if abs(tan(th0))<1e-3
    cons(5)= IsNear(y(2),y(1),tol);
elseif abs(tan(th0))>1e3
    cons(5)= IsNear(x(2),x(1),tol);    
else
    cons(5) = IsNear(y(2),tan(th0)*(x(2)-x(1))+y(1),tol);
end

if abs(tan(th1))<1e-3
    cons(6) = IsNear(y(end-1),y(end));
elseif abs(tan(th1))>1e3
    cons(6) = IsNear(x(end-1),x(end));
else
    cons(6) = IsNear(y(end-1),tan(th1)*(x(end-1)-x(end))+y(end));
end

 cons(7)= all(kappa.^2 <= kappa_max^2);

satisCons=all(cons);
end

