function f = objectiveFunction(x,y,initCOP)
%objectiveFunction Summary of this function goes here
%   Detailed explanation goes here

%% Normal Objective Function
if 0
    t=initCOP.t;
    Pstart=initCOP.Pstart;
    Pend=initCOP.Pend;

    [~,dB,~,~]=BezierCurve(x,y,t);
    dx=dB(:,1);
    dy=dB(:,2);


    f=trapz(t,sqrt(dx.^2+dy.^2));
%% Kappa only objective function
else
    t=initCOP.t;
    [~,~,~,kappa]=BezierCurve(x,y,t);
    f=trapz(t,kappa.^2);
end

end

