function [satisCons] = checkConstraints(x,y,initCOP)
%CHECKCONSTRAINTS Summary of this function goes here
%   Detailed explanation goes here

t=initCOP.t;
Pstart=initCOP.Pstart;
Pend=initCOP.Pend;
kappa_max=initCOP.kappa_max;

x1=Pstart(1); y1=Pstart(2); th1=Pstart(3);
xend=Pend(1); yend=Pend(2); thend=Pend(3);

[~,~,~,kappa]=BezierCurve(x,y,t);

cons=boolean(zeros(7,1));
tol=1e-2;

cons(1)=IsNear(x1,x(1),tol);
cons(2)=IsNear(y(1),y1,tol);
cons(3)=IsNear(x(end),xend,tol);
cons(4)=IsNear(y(end),yend,tol);

if abs(tan(th1))<1e-3
    cons(5)= IsNear(y(2),y(1),tol);
elseif abs(tan(th1))>1e3
    cons(5)= IsNear(x(2),x(1),tol);    
else
    cons(5) = IsNear(y(2),tan(th1)*(x(2)-x(1))+y(1),tol);
end

if abs(tan(thend))<1e-3
    cons(6) = IsNear(y(end-1),y(end));
elseif abs(tan(thend))>1e3
    cons(6) = IsNear(x(end-1),x(end));
else
    cons(6) = IsNear(y(end-1),tan(thend)*(x(end-1)-x(end))+y(end));
end

cons(7)= all(kappa.^2 <= kappa_max^2);

cons(8)=not(IsNear(trapz(t,kappa.^2),0,1e-3));

satisCons=all(cons);
end

