function [B] = BezierCurve1(P,varargin)
%BEZIERCURVECUBIC Summary of this function goes here
%   Detailed explanation goes here

if nargin == 1
    t = linspace(0,1,101)';
else
    t=varargin{1};
end

BezierBasis=[-1,1;1,0];

if length(P) ~= 2
    disp('error in dimentions : you need 4 control points for a Lineair Bézier Curve')
    return
end

Px=P(:,1);
Py=P(:,2);

B=[polyval(BezierBasis*Px,t) polyval(BezierBasis*Py,t)];
end

