function [ Path_Circ,Path_Cloth ] = sortPathPlot(Path)
%[ Path_Circ,Path_Cloth ] = sortPathPlot(Path) Summary of this function goes here
%   Detailed explanation goes here

%     tol=1e-3;
%     th0_all=[MotionPrem.th0].';
%     x1_all=[MotionPrem.x1].';
%     y1_all=[MotionPrem.y1].';
%     th1_all=[MotionPrem.th1].';
%     
%     
%     th0_show=IsNear(rem(th0_all,dth_show),0,tol); % boolean
%     x1_show=IsNear(rem(x1_all,dx_show),0,tol);
%     y1_show=IsNear(rem(y1_all,dx_show),0,tol);
%     th1_show=IsNear(rem(th1_all,dth_show),0,tol);
%     showSelected=logical(x1_show .* y1_show .* th1_show);
%     MotionPremShow=MotionPrem(showSelected);
    
    dk_show=[Path.dk];
    cteK=IsNear(dk_show,0,1e-5).';
    notCteK=~(cteK);
    
    Path_Circ=Path(cteK);
    Path_Cloth=Path(notCteK);
end

