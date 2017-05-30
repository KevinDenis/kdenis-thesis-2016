function LSL=FreeAllPaths(LSL)
%LSL=FreeAllPaths(LSL)
for ii=1:length(LSL)
    LSL(ii).free=true;
    LSL(ii).idxBlocked=length(LSL(ii).X)+1; % by defining the idxBlocked longer as length(X), the algorithm nows it is NOT blocked
end
end