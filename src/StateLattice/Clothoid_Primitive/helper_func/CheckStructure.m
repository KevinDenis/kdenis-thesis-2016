function CheckStructure(MotionStructure)
%CHECKSTRUCTURE Summary of this function goes here
%   Detailed explanation goes here

for ii=1:length(MotionStructure)
    
    if ~IsNear(MotionStructure(ii).X(1), MotionStructure(ii).x0,1e-3)
        disp('error x0')
    end    
    
    if ~IsNear(MotionStructure(ii).X(end), MotionStructure(ii).x1,1e-3)
        disp('error x1')
    end 
    
    if ~IsNear(MotionStructure(ii).Y(1), MotionStructure(ii).y0,1e-3)
        disp('error y0')
    end    
    
    if ~IsNear(MotionStructure(ii).Y(end), MotionStructure(ii).y1,1e-3)
        disp('error y1')
    end
    
    if ~IsNear(MotionStructure(ii).TH(1), MotionStructure(ii).th0,1e-3)
        disp('error th0')
    end    
    
    if ~IsNear(MotionStructure(ii).TH(end), MotionStructure(ii).th1,1e-3)
        disp('error th1')
    end
    
%     dX=DGradient(MotionStructure(ii).X);
%     dY=DGradient(MotionStructure(ii).Y);
% 
%     TH_calc=wrapToPi(atan2(dY,dX));
%     TH_calc(TH_calc==-pi)=pi;
%     
%     TH=MotionStructure(ii).TH;
%    
%     startIdx=round(length(TH)*0.2);
%     endIdx=length(TH)-startIdx;
%     if any(~IsNear(TH(startIdx:endIdx)-TH_calc(startIdx:endIdx),0,1e-3))
%         disp(ii)
%         disp('error TH')
%         plot([TH TH_calc])
%         pause()
%     end

end
end

