function f = costFunction(x,y,alpha)
%COSTFUNCTION Summary of this function goes here
%   Detailed explanation goes here
n=101;
t=linspace(0,1,n)';
P=[x y];    
[~,dB,~,kappa]=BezierCurve(P,t);
dxx=dB(:,1);
dyy=dB(:,2);
ds=cumtrapz(t,sqrt(dxx.^2+dyy.^2));
kappaFactor=trapz(ds,kappa.^2);
distFactor=ds(end);
f=kappaFactor + alpha*distFactor;
end

