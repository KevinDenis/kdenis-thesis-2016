function [satisCons] = checkConstraints(x,y,initCOP)
%CHECKCONSTRAINTS Summary of this function goes here
%   Detailed explanation goes here

t=initCOP.t;
Pstart=initCOP.Pstart;
Pend=initCOP.Pend;
kappa_max=initCOP.kappa_max;

x1=Pstart(1); y1=Pstart(2); theta1=Pstart(3);
xn=Pend(1); yn=Pend(2); thetan=Pend(3);

[~,~,~,kappa]=BezierCurve(x,y,t);

cons=boolean(zeros(7,1));
tol=1e-2;

cons(1)=IsNear(x1,x(1),tol);
cons(2)=IsNear(y(1),y1,tol);
cons(3)=IsNear(x(end),xn,tol);
cons(4)=IsNear(y(end),yn,tol);

if abs(tan(theta1))<1e3
    cons(5)= IsNear(y(2),tan(theta1)*(x(2)-x(1))+y(1),tol);
else
    cons(5) = IsNear(x(2),x(1),tol);
end

if abs(tan(thetan))<1e3
    cons(6) = IsNear(y(end-1),tan(thetan)*(x(end-1)-xn)+yn,tol);
else
    cons(6) = IsNear(x(end-1),x(end),tol);
end

cons(7)= all(kappa.^2 <= kappa_max^2);

satisCons=all(cons);
end

