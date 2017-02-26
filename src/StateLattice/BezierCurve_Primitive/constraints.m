function g = constraints(x,y,initCOP)
%CONSTRAINTS Summary of this function goes here
%   Detailed explanation goes here
t=initCOP.t;
Pstart=initCOP.Pstart;
Pend=initCOP.Pend;
kappa_max=initCOP.kappa_max;

x1=Pstart(1); y1=Pstart(2); th1=Pstart(3);
xend=Pend(1); yend=Pend(2); thend=Pend(3);

[~,dB,~,kappa]=BezierCurve(x,y,t);

dx=dB(:,1);
dy=dB(:,2);

g{1}=[x(1) y(1) x(end) y(end)]'==[x1 y1 xend yend]';

if abs(tan(th1))<1e-3
    g{2}= y(2) == y(1);
elseif abs(tan(th1))>1e3
    g{2} = x(2)== x(1);
else
    g{2} = y(2) == tan(th1)*(x(2)-x(1))+y(1);
end

if abs(tan(thend))<1e-3
    g{3}= y(end-1) == y(end);
elseif abs(tan(thend))>1e3
    g{3} = x(end-1) == x(end);
else
    g{3} = y(end-1) == tan(thend)*(x(end-1)-x(end))+y(end);
end

if tan(th1)>=0
    g{4} = x(2) >= x(1);
    g{5} = y(2) >= y(1);
else
    g{4} = x(2) >= x(1);
    g{5} = y(2) <= y(1);
end

if tan(thend)>=0
    g{6} = x(end-1) <= x(end);
    g{7} = y(end-1) <= y(end);
else
    g{6} = x(end-1) <= x(end);
    g{7} = y(end-1) >= y(end);
end

g{8}= kappa.^2 <= kappa_max^2;

end