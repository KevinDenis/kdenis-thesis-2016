function f = costFunction(x,y,alpha)
%COSTFUNCTION Summary of this function goes here
%   Detailed explanation goes here
%     import casadi.*
    n=101;
    t=linspace(0,1,n)';
    P=[x y];    
    [~,dB,~,kappa]=BezierCurve(P,t);
    dxx=dB(:,1);
    dyy=dB(:,2);
    kappaFactor=trapz(t,kappa.^2);
    distFactor=trapz(t,sqrt(dxx.^2+dyy.^2));
    f=kappaFactor + alpha*distFactor;
end

