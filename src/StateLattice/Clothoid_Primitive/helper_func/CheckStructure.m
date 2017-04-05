function CheckStructure(Path)
%CheckStructure(Path) Summary of this function goes here
%   Detailed explanation goes here

for ii=1:length(Path)
    
    if ~IsNear(Path(ii).X(1), Path(ii).x0,1e-3)
        disp('error x0')
    end    
    
    if ~IsNear(Path(ii).X(end), Path(ii).x1,1e-3)
        disp('error x1')
    end 
    
    if ~IsNear(Path(ii).Y(1), Path(ii).y0,1e-3)
        disp('error y0')
    end    
    
    if ~IsNear(Path(ii).Y(end), Path(ii).y1,1e-3)
        disp('error y1')
    end
    
    if ~IsNear(Path(ii).TH(1), Path(ii).th0,1e-3)
        disp('error th0')
    end    
    
    if ~IsNear(Path(ii).TH(end), Path(ii).th1,1e-3)
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

