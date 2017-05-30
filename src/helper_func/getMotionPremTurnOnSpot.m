function [LSL_ToS] = getMotionPremTurnOnSpot(LSL,th0)
%[LSL_ToS] = getMotionPremTurnOnSpot(LSL,th0)
%   Creates turn of the spot motion, starting from th0
%   ToS = turn of the spot

th_res=pi/64;%
dth=pi/4;


TH1=(-3/4*pi:dth:pi); 
TH1(TH1==th0)=[];

n=length(TH1);
LSL_ToS=repmat(LSL(1),n,1);

for ii=1:length(TH1)
    th1=TH1(ii);
    thDiff=angdiff(th0,th1);

    % not used atm, code does not generate CCW and CW turn, just shortest
    if thDiff > 0
        TH_ccw = wrap2Pi(th0+(0:th_res:thDiff));
        TH_cw  = wrap2Pi(th0+(-(0:th_res:wrapTo2Pi(-thDiff))));
        TH=TH_ccw;
    else
        TH_ccw = wrap2Pi(th0+(0:th_res:wrapTo2Pi(thDiff)));
        TH_cw  = wrap2Pi((th0+(0:-th_res:thDiff))); 
        TH=TH_cw;
    end

    K=sign(thDiff)*inf(size(TH));
    k=sign(thDiff)*inf;

    LSL_ToS(ii).x0=0;
    LSL_ToS(ii).y0=0; 
    LSL_ToS(ii).th0=th0; 
    LSL_ToS(ii).x1=0;
    LSL_ToS(ii).y1=0;
    LSL_ToS(ii).th1=th1;
    LSL_ToS(ii).TH=TH.';
    LSL_ToS(ii).X  = zeros(size(TH)).';
    LSL_ToS(ii).Y  = zeros(size(TH)).';
    LSL_ToS(ii).S = zeros(size(TH)).';
    LSL_ToS(ii).K = K.';
    LSL_ToS(ii).k  = k;
    LSL_ToS(ii).dk = 0;
    LSL_ToS(ii).Ltot=0;
    LSL_ToS(ii).intK=inf;
    LSL_ToS(ii).pathCost=abs(3*thDiff);
    LSL_ToS(ii).free=true;
end

end
