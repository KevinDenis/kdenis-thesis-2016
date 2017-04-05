function [SL_Cleaned] = CleanupLooseStarts(SL)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%   Note :
%   * This relies on the fact that a cloth connection is max 2
%   * This could be done _much_ faster, by not using end points and adding
%   by bulk lone start points
voxel=[[SL.x0].',[SL.y0].',[SL.th0].' [SL.x1].',[SL.y1].',[SL.th1].'];

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

SL_Cleaned=SL;
if toDelete~=0
    SL_Cleaned(toDelete)=[];
end
end