function g = constraints(x,y,initCOP)
%CONSTRAINTS Summary of this function goes here
%   Detailed explanation goes here
t=initCOP.t;
Pstart=initCOP.Pstart;
Pend=initCOP.Pend;
kappa_max=initCOP.kappa_max;

x1=Pstart(1); y1=Pstart(2); theta1=Pstart(3);
xn=Pend(1); yn=Pend(2); thetan=Pend(3);

[~,~,~,kappa]=BezierCurve(x,y,t);

g{1}=[x(1) y(1) x(end) y(end)]==[x1 y1 xn yn];

if abs(tan(theta1))<1e-2
    g{2}= y(2) == y(1);
elseif abs(tan(theta1))>1e2
    g{2} = x(2)== x(1);
else
    g{2}= y(2) == tan(theta1)*(x(2)-x(1))+y(1);
end

if abs(tan(thetan))<1e-2
    g{3}= y(end-1) == y(end);
elseif abs(tan(thetan))>1e2
    g{3} = x(end-1) == x(end);
else
    g{3} = y(end-1) == tan(thetan)*(x(end-1)-x(end))+y(end);
end

g{4}= kappa.^2 <= kappa_max^2;
end