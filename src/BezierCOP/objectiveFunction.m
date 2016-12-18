function f = objectiveFunction(x,y,initCOP)
%objectiveFunction Summary of this function goes here
%   Detailed explanation goes here
t=initCOP.t;
Pstart=initCOP.Pstart;
Pgoal=initCOP.Pgoal;
weights=initCOP.weights;
kappa_max=initCOP.kappa_max;
P=[x y];
d_eg=Pgoal(1:2)-P(end,:);
d_sg=Pgoal(1:2)-Pstart(1:2);

   
[~,dB,~,kappa]=BezierCurve(x,y,t);
dx=dB(:,1);
dy=dB(:,2);

maxLength=norm(d_sg,1); % manhattan distance
maxCurvature=kappa_max.^2; % t=1 !
maxAngle=pi;

kappaFactor=trapz(t,kappa.^2)/maxCurvature;
distFactor=(sqrt(d_eg(1)^2+d_eg(2)^2))/maxLength;
lengthFactor=trapz(t,sqrt(dx.^2+dy.^2))/maxLength;
angleFactor=abs(Pgoal(3)-atan2(dB(end,2),dB(end,1)))/maxAngle;

f=weights(1)*kappaFactor + weights(2)*lengthFactor + weights(3)*distFactor + weights(4)*angleFactor;
end

