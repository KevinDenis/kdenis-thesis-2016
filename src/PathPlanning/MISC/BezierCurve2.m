function [B] = BezierCurve2(P,varargin)
%BEZIERCURVECUBIC Summary of this function goes here
%   Detailed explanation goes here

if nargin == 1
    t = linspace(0,1,101)';
else
    t=varargin{1};
end

BezierBasis=[1,-2,1;-2,2,0;1,0,0];

if length(P) ~= 3
    disp('error in dimentions : you need 3 control points for a Quadratic Bézier Curve')
    return
end

Px=P(:,1);
Py=P(:,2);

B=[polyval(BezierBasis*Px,t) polyval(BezierBasis*Py,t)];
end

