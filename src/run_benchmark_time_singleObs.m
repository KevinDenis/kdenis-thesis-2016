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

stepSize=20;

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
histogram(dt_cloth,'FaceAlpha',1);
histogram(dt_circ,'FaceAlpha',1);
xlabel('Excecution time [msec]')
ylabel('Occurance')
l=legend('Clothoidal LPT','Circular LPT','Location','SE');
set(l,'FontSize',12);
set(gca,'FontSize',12)
hold off
saveCurrentFigure('TimeBench_SingleObs_hist')

figure()
hold on
plot(n_paths_cloth,dt_cloth,'.');
plot(n_paths_circ,dt_circ,'.');
xlabel('Number of paths affected')
ylabel('Excecution time [msec]')
l=legend('Clothoidal LPT','Circular LPT','Location','SE');
set(l,'FontSize',12);
set(gca,'FontSize',12)
hold off
saveCurrentFigure('TimeBench_SingleObs_PathInfluence')


mean_dt_circ=mean(dt_circ);
mean_n_path_circ = mean(n_paths_circ);
mean_dt_per_path_circ = mean(dt_circ./n_paths_circ)*1000;

mean_dt_cloth=mean(dt_cloth);
mean_n_path_cloth = mean(n_paths_cloth);
mean_dt_per_path_cloth = mean(dt_cloth./n_paths_cloth)*1000;


fprintf('\nmean circular LPT time: %1.3f msec\n',mean_dt_circ)
fprintf('mean path circular LPT: %3.0f\n',mean_n_path_circ)
fprintf('mean circular LPT time / path: %3.0f µsec\n\n',mean_dt_per_path_circ)

fprintf('mean clothoidal LPT time: %1.3f msec\n',mean_dt_cloth)
fprintf('mean path clothoidal LPT: %3.0f\n',mean_n_path_cloth)
fprintf('mean clothoidal LPT time / path: %3.0f µsec\n\n',mean_dt_per_path_cloth)

fprintf('clothoidal LPT is %2.0f%% slower compared to the circular LPT\n',mean_dt_cloth/mean_dt_circ*100)
fprintf('clothoidal LPT has %2.0f%% more path compared to the circular LPT\n',mean_n_path_cloth/mean_n_path_circ*100)
fprintf('clothoidal LPT is %2.0f%% time/path slower compared to the circular LPT\n\n',mean_dt_per_path_cloth/mean_dt_per_path_circ*100)

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
dt=toc*1000;
n_paths=length(IDOccPaths);
end