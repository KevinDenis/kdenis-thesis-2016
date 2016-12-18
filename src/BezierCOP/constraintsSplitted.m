function g = constraintsSplitted(x,y,a,b,initCOP)
%CONSTRAINTS Summary of this function goes here
%   Detailed explanation goes here
t=initCOP.t;
Pstart=initCOP.Pstart;
kappa_max=initCOP.kappa_max;
obs=initCOP.obs;
zeroToZMatrix=initCOP.zeroToZMatrix;
zToTMatrix=initCOP.zToTMatrix;
x1=Pstart(1); y1=Pstart(2); theta1=Pstart(3);

[B,dB,~,kappa]=BezierCurve(x,y,t);
dx=dB(:,1);
dy=dB(:,2);
x_start=zeroToZMatrix*x;
x_end=zToTMatrix*x;
y_start=zeroToZMatrix*y;
y_end=zToTMatrix*y;

s=trapz(t,sqrt(dx.^2+dy.^2));

g{1}=[x(1) y(1)]==[x1 y1];
g{2}= x(2) >= x(1);
g{3}= y(2) == tan(theta1)*(x(2)-x(1))+y(1);
g{4}= s.^2 <= 2^2;
g{5}= kappa.^2 <= kappa_max^2;
g{6}=[x_start y_start]*a(:,1)-b(1)<=0;
g{7}=[x_end y_end]*a(:,2)-b(2)<=0;
g{8}=obs*a(:,1)-b(1)>=0;
g{9}=obs*a(:,2)-b(2)>=0;
g{10}=norm_2(a(:,1))<=1;
g{11}=norm_2(a(:,2))<=1;
end