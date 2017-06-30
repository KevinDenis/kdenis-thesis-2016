% initWorkspace
clc
close all
if ~exist('LSLset', 'var'); load('LSLset.mat'); end
if ~exist('grid_XY', 'var'); load('grid_XY.mat'); end
if ~exist('LSL_cloth', 'var'); load('LSL_cloth.mat'); LSL_cloth = LSL; end
if ~exist('ObstacleTable_cloth', 'var'); load('ObstacleTable_cloth.mat'); ObstacleTable_cloth=ObstacleTable; end
if ~exist('XY_ObsTable_cloth', 'var'); load('XY_ObsTable_cloth.mat'); XY_ObsTable_cloth=XY_ObsTable; end
if ~exist('LSL_circ', 'var'); load('LSL_circ.mat'); LSL_circ = LSL; end
if ~exist('ObstacleTable_circ', 'var'); load('ObstacleTable_circ.mat'); ObstacleTable_circ=ObstacleTable; end
if ~exist('XY_ObsTable_circ', 'var'); load('XY_ObsTable_circ.mat'); XY_ObsTable_circ=XY_ObsTable; end


n_circ=length(ObstacleTable_circ);
n_cloth=length(ObstacleTable_cloth);

stepSize=10;

n=min(n_circ,n_cloth);
n=floor(n/stepSize);
dt_circ=zeros(n,1);
n_paths_circ=zeros(n,1);
dt_cloth=zeros(n,1);
n_paths_cloth=zeros(n,1);
for ii=1:n
    jj=ii*stepSize;
    XY_obs_circ=XY_ObsTable_circ(jj,:);
    XY_obs_cloth=XY_ObsTable_cloth(jj,:);
    [dt_circ_ii,n_paths_circ_ii]=UpdateStateLattice(XY_obs_circ,LSL_circ,ObstacleTable_circ,XY_ObsTable_circ);
    [dt_cloth_ii,n_paths_cloth_ii]=UpdateStateLattice(XY_obs_cloth,LSL_cloth,ObstacleTable_cloth,XY_ObsTable_cloth);
    dt_circ(ii)=dt_circ_ii;
    n_paths_circ(ii)=n_paths_circ_ii;
    dt_cloth(ii)=dt_cloth_ii;
    n_paths_cloth(ii)=n_paths_cloth_ii;
end

[dt_cloth,idx_cloth_S]=sort(dt_cloth);
n_paths_cloth=n_paths_cloth(idx_cloth_S);
dt_cloth(end-stepSize:end)=[];
n_paths_cloth(end-stepSize:end)=[];


[dt_circ,idx_circ_S]=sort(dt_circ);
n_paths_circ=n_paths_circ(idx_circ_S);
dt_circ(end-10:end)=[];
n_paths_circ(end-10:end)=[];

figure()
hold on
histogram(dt_cloth);
histogram(dt_circ);

figure()
hold on
plot(n_paths_cloth,dt_cloth,'.');
plot(n_paths_circ,dt_circ,'.');

%% Functions
function [dt,n_paths]=UpdateStateLattice(XY_ObsGrid,LSL,ObstacleTable,XY_Table)
xy_obs_end=XY_ObsGrid(end,:);
tic
% idxRow=find(ismember(XY_Table,XY_ObsGrid,'rows'));
idxRow=findrow_mex(XY_Table,xy_obs_end);
IDOccPaths = [ObstacleTable(idxRow).ID];
IdxOccPaths = [ObstacleTable(idxRow).Idx];
for jj=1:length(IDOccPaths)
    LSL(IDOccPaths(jj)).free=false;
    if isempty(LSL(IDOccPaths(jj)).idxBlocked) || LSL(IDOccPaths(jj)).idxBlocked > IdxOccPaths(jj)
        LSL(IDOccPaths(jj)).idxBlocked=IdxOccPaths(jj);
    end
end
dt=toc;
n_paths=length(IDOccPaths);
end