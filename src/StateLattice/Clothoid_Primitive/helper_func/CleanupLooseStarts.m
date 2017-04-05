function [Path_Cleaned] = CleanupLooseStarts(Path)
%[Path_Cleaned] = CleanupLooseStarts(Path) Summary of this function goes here
%   Detailed explanation goes here
%   Note :
%   * This relies on the fact that a cloth connection is max 2
%   * This could be done _much_ faster, by not using end points and adding
%   by bulk lone start points

voxel=[[Path.x0].',[Path.y0].',[Path.th0].' [Path.x1].',[Path.y1].',[Path.th1].'];
toDelete=zeros(1);
nn=1;
for ii=1:length(voxel)
    voxel_ii=voxel(ii,:);
    idxOriginToStart=findVector(voxel,[0 0 0 voxel_ii(1:3)]);
    if isnan(idxOriginToStart) &&  ~(voxel_ii(1)==0 && voxel_ii(2)==0 && voxel_ii(3)==0)
        toDelete(nn)=ii;
        nn=nn+1;
    end
end

Path_Cleaned=Path;
if toDelete~=0
    Path_Cleaned(toDelete)=[];
end
end