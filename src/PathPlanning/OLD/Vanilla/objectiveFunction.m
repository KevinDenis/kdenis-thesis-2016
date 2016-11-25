function f = objectiveFunction(x,y,alpha,Pstart,Pgoal)
%COSTFUNCTION Summary of this function goes here
%   Detailed explanation goes here
x1=Pstart(1); y1=Pstart(2); theta1=Pstart(3);
xg=Pgoal(1); yg=Pgoal(2); thetag=Pgoal(3);

kappa_max=2;


n=101;
t=linspace(0,1,n)';
P=[x y];    
[~,dB,~,kappa]=BezierCurve(P,t);
dx=dB(:,1);
dy=dB(:,2);
ds=cumtrapz(t,sqrt(dx.^2+dy.^2));

kappaFactor_max=ds(end)*kappa_max;
kappaFactor=trapz(ds,kappa.^2)/kappaFactor_max;

length_min=Norm2(Pgoal(1:2)-Pstart(1:2));

lengthFactor=ds(end)/length_min;

distFactor=Norm2([xg yg]-[x(end) y(end)]);


angleFactor=0*abs(thetag-atan2(y(end)-y(end-1),x(end)-x(end-1)));

f=kappaFactor + alpha*lengthFactor+0.5*alpha*distFactor+alpha*angleFactor;

end

