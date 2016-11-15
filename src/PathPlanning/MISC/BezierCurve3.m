function [B,dB,ddB] = BezierCurve3(P,varargin)
%BEZIERCURVECUBIC Summary of this function goes here
%   Detailed explanation goes here

if nargin == 1
    t = linspace(0,1,101)';
else
    t=varargin{1};
end

BezierBasis=[-1,3,-3,1;3,-6,3,0;-3,3,0,0;1,0,0,0];

if length(P) ~= 4
    disp('error in dimentions : you need 4 control points for a Cubic Bézier Curve')
    return
end

n=length(P)-1; % order
Px=P(:,1);
Py=P(:,2);
dP=[diff(P(:,1)),diff(P(:,2))];
ddP=[diff(dP(:,1)),diff(dP(:,2))];

B=[polyval(BezierBasis*Px,t) polyval(BezierBasis*Py,t)];
dB=n*BezierCurve2(dP,t);
ddB=(n-1)*n*BezierCurve1(ddP,t);

end

