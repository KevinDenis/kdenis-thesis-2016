function [Path_Cleaned] = CleanupStateLattice(Path)
%[Path_Cleaned] = CleanupStateLattice(Path) Summary of this function goes here
%   Detailed explanation goes here
%   Note :
%   * This relies on the fact that a cloth connection is max 2

% check if there is not 2 times start end (impossible normaly)
voxel=[[Path.x0].',[Path.y0].',[Path.th0].' [Path.x1].',[Path.y1].',[Path.th1].'];
[~,idx,~]=unique(voxel,'rows');
Path=Path(idx);
voxel=voxel(idx,:);
   
toDelete=zeros(1);
nn=1;
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
            toDelete(nn)=ii;
            nn=nn+1;
        end
    end
end

Path_Cleaned=Path;
Path_Cleaned(toDelete)=[];

% assign path ID, could this be faster ?
for ii=1:length(Path_Cleaned)
    Path_Cleaned(ii).ID=ii;
end
end