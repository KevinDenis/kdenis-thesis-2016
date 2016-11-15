function [B,dB,ddB] = BezierCurve4(P,varargin)
%BEZIERCURVEQUATIC Summary of this function goes here
%   Detailed explanation goes here


if nargin == 1
    t = linspace(0,1,101)';
else
    t=varargin{1};
end

BezierBasis=[1,-4,6,-4,1;-4,12,-12,4,0;6,-12,6,0,0;-4,4,0,0,0;1,0,0,0,0];

if length(P) ~= 5
    disp('error in dimentions : you need 5 control points for a Quartic Bézier Curve')
    return
end

n=length(P)-1; % order
Px=P(:,1);
Py=P(:,2);
dP=[diff(P(:,1)),diff(P(:,2))];
ddP=[diff(dP(:,1)),diff(dP(:,2))];

B=[polyval(BezierBasis*Px,t) polyval(BezierBasis*Py,t)];
dB_temp=BezierCurve3(dP,t);
dB=n*dB_temp;
ddB_temp=BezierCurve2(ddP,t);
ddB=(n-1)*n*ddB_temp;
end

