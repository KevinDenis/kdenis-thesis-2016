function [x_rot,y_rot] = Rotate0Theta(xx,yy,th)
%ROTATE0THETA Summary of this function goes here
%   Detailed explanation goes here
% create a matrix of these points, which will be useful in future calculations
v = [xx.';yy.'];
R = [cos(th) -sin(th); sin(th) cos(th)];
vo = R*v;           % apply the rotation about the origin
x_rot = vo(1,:).';
y_rot = vo(2,:).';
end

