function [LSL_FwdRev] = AddReverseDirection(LSL_Fwd)
%[LSL_FwdRev] = AddReverseDirection(LSL_Fwd)
%   Adds reverse direction of previously calculated forward motion
%   Performed by simple coordinate transformation
offsetID=LSL_Fwd(end).ID;
LSL_Rev=removeToSFromLSL(LSL_Fwd); % remove Turn On the Spot from LSL for reverse direction
n=length(LSL_Rev);
for ii=1:n
    ID_ii = offsetID + ii;
    LSL_Rev(ii).x0  = -LSL_Rev(ii).x0;
    LSL_Rev(ii).th0 =  wrap2Pi(-LSL_Rev(ii).th0);
    LSL_Rev(ii).x1  = -LSL_Rev(ii).x1;
    LSL_Rev(ii).th1 =  wrap2Pi(-LSL_Rev(ii).th1);
    LSL_Rev(ii).k   = -LSL_Rev(ii).k;
    LSL_Rev(ii).dk  = -LSL_Rev(ii).dk;
    LSL_Rev(ii).X   = -LSL_Rev(ii).X;
    LSL_Rev(ii).TH  =  wrap2Pi(-LSL_Rev(ii).TH);
    LSL_Rev(ii).ID  =  ID_ii;
    LSL_Rev(ii).pathCost=5*LSL_Rev(ii).pathCost;
end
LSL_FwdRev=[LSL_Fwd;LSL_Rev];
end

