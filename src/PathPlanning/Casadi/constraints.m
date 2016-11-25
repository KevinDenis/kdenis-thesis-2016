function g = constraints(x,y,initCOP)
%CONSTRAINTS Summary of this function goes here
%   Detailed explanation goes here
t=initCOP.t;
Pstart=initCOP.Pstart;
%Pgoal=initCOP.Pgoal;
kappa_max=initCOP.kappa_max;

x1=Pstart(1); y1=Pstart(2); theta1=Pstart(3);
%xg=Pgoal(1); yg=Pgoal(2); thetag=Pgoal(3);


[~,dB,~,kappa]=BezierCurve([x,y],t);
dx=dB(:,1);
dy=dB(:,2);

s=trapz(t,sqrt(dx.^2+dy.^2));

g{1}=[x(1) y(1)]==[x1 y1];
g{2}= x(2) >= x(1);
g{3}= y(2) == tan(theta1)*(x(2)-x(1))+y(1);
g{4}= s.^2 <= 2^2;
g{5}= kappa.^2 <= kappa_max^2;
end

