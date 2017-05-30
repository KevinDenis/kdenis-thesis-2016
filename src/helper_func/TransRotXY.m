function [XY_rot_trans] = TransRotXY(XY,th,x,y)
%[XY_rot_trans] = TransRotXY(XY,th,x,y)
%    Translates XY data by x y then rotates by th
X_trans=XY(:,1)+x;
Y_trans=XY(:,2)+y;
[X_trans_rot,Y_trans_rot]=Rotate0Theta(X_trans,Y_trans,th);
XY_rot_trans=[X_trans_rot Y_trans_rot];
end