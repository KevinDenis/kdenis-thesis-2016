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
% stepSize=500;

n=min(n_circ,n_cloth);
n=floor(n/stepSize);
removeMaxElem = floor(n*0.01);

% Clothoidal LPT
dt_mex_cloth=zeros(n,1);
dt_mat_cloth=zeros(n,1);
n_paths_mat_cloth=zeros(n,1);

% Circular LPT
dt_mex_circ=zeros(n,1);
dt_mat_circ=zeros(n,1);
n_paths_mex_circ=zeros(n,1);
n_paths_mat_circ=zeros(n,1);

hash = randn(size(XY_ObsTable_cloth,2),1);
XY_ObsTable_cloth_hash = round(XY_ObsTable_cloth*hash,6);
XY_ObsTable_circ_hash = round(XY_ObsTable_circ*hash,6);

for ii=1:n
    jj=ii*stepSize;
    
    % Clothoidal LPT
    XY_obs_cloth=XY_ObsTable_cloth(jj,:);
    XY_obs_cloth_hash = round(XY_obs_cloth*hash,6);
    [dt_mat_cloth(ii),n_paths_mat_cloth(ii)] = UpdateStateLatticeMat(XY_obs_cloth_hash,LSL_cloth,ObstacleTable_cloth,XY_ObsTable_cloth_hash);
    
    % Circular LPT
	XY_obs_circ=XY_ObsTable_circ(jj,:);
	XY_obs_circ_hash = round(XY_obs_circ*hash,6);
    [dt_mat_circ(ii),n_paths_mat_circ(ii)]  = UpdateStateLatticeMat(XY_obs_circ_hash,LSL_circ,ObstacleTable_circ,XY_ObsTable_circ_hash);
end

% Clothoidal LPT - mat
[dt_mat_cloth,idx_mat_cloth_S]=sort(dt_mat_cloth);
n_paths_mat_cloth=n_paths_mat_cloth(idx_mat_cloth_S);
dt_mat_cloth(end-removeMaxElem:end)=[];
n_paths_mat_cloth(end-removeMaxElem:end)=[];

% Circular LPT - mat
[dt_mat_circ,idx_mat_circ_S]=sort(dt_mat_circ);
n_paths_mat_circ=n_paths_mat_circ(idx_mat_circ_S);
dt_mat_circ(end-removeMaxElem:end)=[];
n_paths_mat_circ(end-removeMaxElem:end)=[];

%% plots
figure()
hold on
histogram(dt_mat_cloth,'FaceAlpha',1);
histogram(dt_mat_circ,'FaceAlpha',1);
xlabel('Excecution time [msec]')
ylabel('Occurrence')
l=legend('Clothoidal LPT','Circular LPT','Location','NE');
set(l,'FontSize',14);
set(gca,'FontSize',14)
hold off
saveCurrentFigure('TimeBench_SingleObs_mat_hist')

figure()
hold on
plot(n_paths_mat_cloth,dt_mat_cloth,'.');
plot(n_paths_mat_circ,dt_mat_circ,'.');
xlabel('Number of paths affected')
ylabel('Excecution time [msec]')
l=legend('Clothoidal LPT','Circular LPT','Location','SE');
set(l,'FontSize',14);
set(gca,'FontSize',14)
hold off
saveCurrentFigure('TimeBench_SingleObs_mat_PathInfluence')

median_dt_circ=median(dt_mat_circ);
median_n_path_circ = median(n_paths_mat_circ);
median_dt_per_path_circ = median(dt_mat_circ./n_paths_mat_circ)*1000;

median_dt_cloth=median(dt_mat_cloth);
median_n_path_cloth = median(n_paths_mat_cloth);
median_dt_per_path_cloth = median(dt_mat_cloth./n_paths_mat_cloth)*1000;

fprintf('\n==========================')
fprintf('\n=== search results ===')

fprintf('\nmedian circular LPT time: %1.3f msec\n',median_dt_circ)
fprintf('median path circular LPT: %3.0f\n',median_n_path_circ)
fprintf('median circular LPT time / path: %3.0f µsec\n\n',median_dt_per_path_circ)

fprintf('median clothoidal LPT time: %1.3f msec\n',median_dt_cloth)
fprintf('median path clothoidal LPT: %3.0f\n',median_n_path_cloth)
fprintf('median clothoidal LPT time / path: %3.0f µsec\n\n',median_dt_per_path_cloth)

fprintf('clothoidal LPT is %2.0f%% slower compared to the circular LPT\n',median_dt_cloth/median_dt_circ*100)
fprintf('clothoidal LPT has %2.0f%% more path compared to the circular LPT\n',median_n_path_cloth/median_n_path_circ*100)
fprintf('clothoidal LPT is %2.0f%% time/path slower compared to the circular LPT\n\n',median_dt_per_path_cloth/median_dt_per_path_circ*100)
fprintf('\n==========================\n')

%% Functions
function [dt,n_paths]=UpdateStateLatticeMat(XY_ObsGrid,LSL,ObstacleTable,XY_Table)
xy_obs_end=XY_ObsGrid(end,:);
tic
idxRow=find(ismember(XY_Table,xy_obs_end));
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