function [Path] = CleanupLooseStarts(Path)
%[Path_Cleaned] = CleanupLooseStarts(Path) Summary of this function goes here
%   Detailed explanation goes here
%   Note :
%   * This relies on the fact that a cloth connection is max 2
vertices = getStartEndVerticesPath(Path);
for ii=1:size(vertices,1)
    vertex_ii=vertices(ii,:);
    idxOriginToStart=findrow_mex(vertices,[0 0 0 vertex_ii(1:3)]);
    if ~(vertex_ii(1)==0 && vertex_ii(2)==0 && vertex_ii(3)==0)  && ~Path(idxOriginToStart).free  
        Path(ii).free=false;
        Path(ii).idxBlocked=1; % it is blocked at the start, since predecessor is also blocked
    end
end
end