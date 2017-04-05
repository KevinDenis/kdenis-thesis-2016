function [SL_Cleaned] = CleanupStateLattice(SL)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%   Note :
%   * This relies on the fact that a cloth connection is max 2

% check if there is not 2 times start end (impossible normaly)
voxel=[[SL.x0].',[SL.y0].',[SL.th0].' [SL.x1].',[SL.y1].',[SL.th1].'];
[~,idx,~]=unique(voxel,'rows');
SL=SL(idx);
voxel=voxel(idx,:);
   
toDelete=[];
for ii=2:length(voxel)
    % gather all voxels before current voxel
    prevVoxelSet=voxel(1:ii-1,:); 
    prevStartVoxelSet=voxel(1:ii-1,1:3);
    prevEndVoxelSet=voxel(1:ii-1,4:6);
    % gather current voxel
    voxel_ii=voxel(ii,:);    
    startVoxel_ii=voxel_ii(1:3);
    endVoxel_ii=voxel_ii(4:6);
    
    idxEndPrevVox=findVector(prevEndVoxelSet,endVoxel_ii);
    if ~isnan(idxEndPrevVox)
        prevEndVoxels_start=prevStartVoxelSet(idxEndPrevVox,:);
        idxStartOldToStartNew=findVector(prevVoxelSet,[prevEndVoxels_start startVoxel_ii]);
        if ~isnan(idxStartOldToStartNew)
            toDelete=[toDelete;ii];
        end
    end
end

SL_Cleaned=SL;
SL_Cleaned(toDelete)=[];

% assign path ID, could this be faster ?
for ii=1:length(SL_Cleaned)
    SL_Cleaned(ii).ID=ii;
end
end