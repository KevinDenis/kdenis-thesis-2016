initWorkspace
load('LSL_cloth.mat')

maxSize=1;
maxSizeIdx=0;

for ii=1:length(LSL)
    
    size_ii=size(LSL(ii).PathOccXY,1);
    
    if size_ii > maxSize
        maxSize=size_ii;
        maxSizeIdx = ii;
        
    end
    
    
    
end


