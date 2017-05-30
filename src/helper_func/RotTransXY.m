function [XY_rot_trans] = RotTransXY(XY,th,x,y)
%[XY_rot_trans] = RotTransXY(XY,th,x,y)
%   Rotates XY data by th and then translates it by x y
[X_rot,Y_rot]=Rotate0Theta(XY(:,1),XY(:,2),th);
X_rot_trans=X_rot+x;
Y_rot_trans=Y_rot+y;
XY_rot_trans=[X_rot_trans Y_rot_trans];
end