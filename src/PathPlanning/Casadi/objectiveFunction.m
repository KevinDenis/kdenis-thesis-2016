function f = objectiveFunction(x,y,initCOP)
%COSTFUNCTION Summary of this function goes here
%   Detailed explanation goes here

t=initCOP.t;
Pstart=initCOP.Pstart;
Pgoal=initCOP.Pgoal;
weights=initCOP.weights;
kappa_max=initCOP.kappa_max;

x1=Pstart(1); y1=Pstart(2); theta1=Pstart(3);
xg=Pgoal(1); yg=Pgoal(2); thetag=Pgoal(3);


P=[x y];    
[~,dB,~,kappa]=BezierCurve(P,t);
dx=dB(:,1);
dy=dB(:,2);

maxLength=abs((xg-x1)+(yg-y1));
maxCurvature=kappa_max;
maxAngle=pi;

kappaFactor=trapz(t,kappa.^2)/maxCurvature;
distFactor=sqrt((xg-x(end))^2+(yg-y(end))^2)/maxLength;
lengthFactor=trapz(t,sqrt(dx.^2+dy.^2))/maxLength;
angleFactor=abs(thetag-atan2(y(end)-y(end-1),x(end)-x(end-1)))/maxAngle;

f=weights(1)*kappaFactor + weights(2)*lengthFactor + weights(3)*distFactor + weights(4)*angleFactor;
end

