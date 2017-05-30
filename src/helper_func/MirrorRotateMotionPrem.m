function [MotionPrem] = MirrorRotateMotionPrem(MotionPremQ1)
%[MotionPrem] = MirrorRotateMotionPrem(MotionPremQ1)
% TODO :
% * Make it faster by using original MotionPremStructQ1. This will envolve
% a mixed build up, this might be slow also ... certainly to program
% npts=100;

MotionPremQ2=MotionPremQ1;
for ii=1:length(MotionPremQ2)
    MotionPremQ2(ii).th0    = wrap2Pi(pi-MotionPremQ1(ii).th0);
    MotionPremQ2(ii).x1     =-MotionPremQ1(ii).x1;
    MotionPremQ2(ii).th1    = wrap2Pi(pi-MotionPremQ1(ii).th1);
    MotionPremQ2(ii).k      =-MotionPremQ1(ii).k;
    MotionPremQ2(ii).dk     =-MotionPremQ1(ii).dk;
    MotionPremQ2(ii).X      =-MotionPremQ1(ii).X;
    MotionPremQ2(ii).TH     = wrap2Pi(pi-MotionPremQ1(ii).TH);
end


MotionPremQ3=MotionPremQ1;
for ii=1:length(MotionPremQ3)
    MotionPremQ3(ii).th0    = wrap2Pi(-pi+MotionPremQ1(ii).th0);
    MotionPremQ3(ii).x1     =-MotionPremQ1(ii).x1;
    MotionPremQ3(ii).y1     =-MotionPremQ1(ii).y1;
    MotionPremQ3(ii).th1    = wrap2Pi(-pi+MotionPremQ1(ii).th1);
    MotionPremQ3(ii).X      =-MotionPremQ1(ii).X;
    MotionPremQ3(ii).Y      =-MotionPremQ1(ii).Y;
    MotionPremQ3(ii).TH     =wrap2Pi(-pi+MotionPremQ1(ii).TH); 
end

MotionPremQ4=MotionPremQ1;
for ii=1:length(MotionPremQ4)
    MotionPremQ4(ii).th0    = -wrap2Pi(MotionPremQ1(ii).th0);
    MotionPremQ4(ii).y1     =-MotionPremQ1(ii).y1;
    MotionPremQ4(ii).th1    = -wrap2Pi(MotionPremQ1(ii).th1);
    MotionPremQ4(ii).k      =-MotionPremQ1(ii).k;
    MotionPremQ4(ii).dk     =-MotionPremQ1(ii).dk;
    MotionPremQ4(ii).Y      =-MotionPremQ1(ii).Y;
    MotionPremQ4(ii).TH     =-wrap2Pi(MotionPremQ1(ii).TH);
end

MotionPrem_rot0=[MotionPremQ1;MotionPremQ2;MotionPremQ3;MotionPremQ4];
MotionPrem_rot90=RotTransMotionPrem(MotionPrem_rot0,pi/2,0,0);
MotionPrem=[MotionPrem_rot0;MotionPrem_rot90];

StartEndPost=[[MotionPrem.x0].' [MotionPrem.y0].' [MotionPrem.th0].' ...
              [MotionPrem.x1].' [MotionPrem.y1].' [MotionPrem.th1].'];

[~,idx,~] =  unique(StartEndPost,'rows'); 
MotionPrem=MotionPrem(idx);                  
end