function g = constraints(x,y)
%CONSTRAINTS Summary of this function goes here
%   Detailed explanation goes here
[B,~,~,kappa]=BezierCurve([x,y]);
x1=0;
y1=0;
theta1=0;
xend=2;
yend=2;
thetaend=pi/2;

A1=[-tan(theta1) 1];
Aend=[-tan(thetaend) 1];
b1=-tan(theta1)*x1+y1;
bend=-tan(thetaend)*xend+yend;

A=[[A1 zeros(size(Aend))];[0 0 Aend]];
b=[b1;bend];
X=[x(2) y(2) x(end-1) y(end-1)]';

g{1}=[x(1) x(end)]==[x1 xend];
g{2}=[y(1) y(end)]==[y1 yend];
g{3}= x(1) <= x(2:end-1) <= x(end);
g{4}= y(1) <= y(2:end-1) <= y(end);
g{5}= kappa.^2 <= 2^2;
g{6}= A*X == b;
% g{7}= RectIntersect(obs, [veh(:,1)+B(1,1) veh(:,2)+B(1,2)])==1;

% g{3}= y(2) == tan(theta1)*(x(2)-x1)+y1;
% g{4}= y(end-1) == tan(thetaend)*(x(end-1)-xend)+yend;
% g{3}= A1*[x(2) y(2)]' == b1;
% g{4}= Aend*[x(end-1) y(end-1)]' == bend;
end

