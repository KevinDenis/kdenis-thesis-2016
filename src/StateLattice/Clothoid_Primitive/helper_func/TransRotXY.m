function [XY_rot_trans] = TransRotXY(XY,th,x,y)
%ROTTRANSMOTIONPREM Summary of this function goes here
%   Detailed explanation goes here
X_trans=XY(:,1)+x;
Y_trans=XY(:,2)+y;
[X_trans_rot,Y_trans_rot]=Rotate0Theta(X_trans,Y_trans,th);
XY_rot_trans=[X_trans_rot Y_trans_rot];
end