function [Path] = CleanupLooseStarts(Path)
%[Path_Cleaned] = CleanupLooseStarts(Path) Summary of this function goes here
%   Detailed explanation goes here
%   Note :
%   * This relies on the fact that a cloth connection is max 2
vertices = getStartEndVerticesPath(Path);
for ii=1:size(vertices,1)
    voxel_ii=vertices(ii,:);
    idxOriginToStart=findVector(vertices,[0 0 0 voxel_ii(1:3)]);
    if ~(voxel_ii(1)==0 && voxel_ii(2)==0 && voxel_ii(3)==0)  && ~Path(idxOriginToStart).free  
        Path(ii).free=false;
        Path(ii).idxBlocked=1;
    end
end
end