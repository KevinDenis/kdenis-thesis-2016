function [MotionPremRotTrans] = RotTransMotionPrem(MotionPrem,th,x,y)
%[MotionPremRotTrans] = RotTransMotionPrem(MotionPrem,th,x,y)
%   Detailed explanation goes here
MotionPremRotTrans=MotionPrem;
for ii=1:length(MotionPremRotTrans)
    [X_rot,Y_rot]=Rotate0Theta(MotionPrem(ii).X,MotionPrem(ii).Y,th);
    [x0_rot,y0_rot]=Rotate0Theta(MotionPrem(ii).x0,MotionPrem(ii).y0,th);
    [x1_rot,y1_rot]=Rotate0Theta(MotionPrem(ii).x1,MotionPrem(ii).y1,th);
    MotionPremRotTrans(ii).x0=round(x0_rot+x,2);
    MotionPremRotTrans(ii).y0=round(y0_rot+y,2);
    MotionPremRotTrans(ii).th0= wrap2Pi(MotionPremRotTrans(ii).th0+th);
    MotionPremRotTrans(ii).x1= round(x1_rot+x,2);
    MotionPremRotTrans(ii).y1= round(y1_rot+y,2);
    MotionPremRotTrans(ii).th1= wrap2Pi(MotionPremRotTrans(ii).th1+th);
    MotionPremRotTrans(ii).X=X_rot+x;
    MotionPremRotTrans(ii).Y=Y_rot+y;
    MotionPremRotTrans(ii).TH=wrap2Pi(MotionPremRotTrans(ii).TH+th);
end
end