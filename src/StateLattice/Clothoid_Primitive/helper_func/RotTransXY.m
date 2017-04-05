function [XY_rot_trans] = RotTransXY(XY,th,x,y)
%RotTransXY Summary of this function goes here
%   Detailed explanation goes here
[X_rot,Y_rot]=Rotate0Theta(XY(:,1),XY(:,2),th);
X_rot_trans=X_rot+x;
Y_rot_trans=Y_rot+y;
XY_rot_trans=[X_rot_trans Y_rot_trans];
end