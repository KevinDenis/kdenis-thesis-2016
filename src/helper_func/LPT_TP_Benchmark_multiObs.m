function LPT_TP_Benchmark_multiObs(usePrecomputedData,saveComputedData,saveFigure)

if usePrecomputedData
    load('TimeBench_MultiObs.mat')
else
    if ~exist('LSLset', 'var'); load('LSLset.mat'); end
    if ~exist('grid_XY', 'var'); load('grid_XY.mat'); end
    if ~exist('LSL_cloth', 'var'); load('LSL_cloth.mat'); LSL_cloth = LSL; end
    if ~exist('ObstacleTable_cloth', 'var'); load('ObstacleTable_cloth.mat'); ObstacleTable_cloth=ObstacleTable; end
    if ~exist('XY_ObsTable_cloth', 'var'); load('XY_ObsTable_cloth.mat'); XY_ObsTable_cloth=XY_ObsTable; end
    if ~exist('LSL_circ', 'var'); load('LSL_circ.mat'); LSL_circ = LSL; end
    if ~exist('ObstacleTable_circ', 'var'); load('ObstacleTable_circ.mat'); ObstacleTable_circ=ObstacleTable; end
    if ~exist('XY_ObsTable_circ', 'var'); load('XY_ObsTable_circ.mat'); XY_ObsTable_circ=XY_ObsTable; end

    selectMap = 3; % RobotLab_Elevator=1, RobotLab=2, RobotLab_ZoomEntrance=3, Elevator=4

    [userRobotPose,XY_occ_lab]=getUseDefPose(selectMap);
    
    n_des = 1000;
    n_space = round(n_des^(1/3));
    n_angle = round(n_des/n_space^2);
    x_var=linspace(-0.5,0.5,n_space);
    y_var=x_var;
    th_var=linspace(-pi/2,pi/2,n_angle);

    robotPose_vec = getAllComb(x_var,y_var,th_var);

    allRobotPoses = repmat(userRobotPose,size(robotPose_vec,1),1)+robotPose_vec;

    n=size(robotPose_vec,1);
    removeMaxElem = floor(n*0.01);
    delMaxdT=floor(n/10);
    dt_circ=zeros(n,1);
    n_obs_circ=zeros(n,1);
    dt_cloth=zeros(n,1);
    n_obs_cloth=zeros(n,1);

    ppm = ParforProgressStarter2('Calculating...', n, 0.1, 0, 1, 1);
    
    hash = randn(size(XY_ObsTable_cloth,2),1);
    XY_ObsTable_cloth_hash = round(XY_ObsTable_cloth*hash,6);
    XY_ObsTable_circ_hash = round(XY_ObsTable_circ*hash,6);
    
    for ii=1:n
        XY_occ_lab_R=getXY_occ_lab_R(XY_occ_lab,allRobotPoses(ii,:));
        XY_occ_lab_R_hash = round(XY_occ_lab_R*hash,6);
        [dt_cloth(ii), n_obs_cloth(ii)]=AdjustPathLength(LSL_cloth,ObstacleTable_cloth,XY_ObsTable_cloth_hash,XY_occ_lab_R_hash);
        [dt_circ(ii), n_obs_circ(ii)]=AdjustPathLength(LSL_circ,ObstacleTable_circ,XY_ObsTable_circ_hash,XY_occ_lab_R_hash);
        ppm.increment(ii);
    end
    delete(ppm);
    
    % Clothoidal LPT - mat
    [dt_cloth,idx_mat_cloth_S]=sort(dt_cloth);
    n_obs_cloth=n_obs_cloth(idx_mat_cloth_S);
    dt_cloth(end-removeMaxElem:end)=[];
    n_obs_cloth(end-removeMaxElem:end)=[];

    % Circular LPT - mat
    [dt_circ,idx_mat_circ_S]=sort(dt_circ);
    n_obs_circ=n_obs_circ(idx_mat_circ_S);
    dt_circ(end-removeMaxElem:end)=[];
    n_obs_circ(end-removeMaxElem:end)=[];
     
    if saveComputedData
        save('data_mat/TimeBench_MultiObs.mat','dt_circ','n_obs_circ','dt_cloth','n_obs_cloth')
    end

end

figure()
hold on
histogram(dt_cloth,'FaceAlpha',1);
histogram(dt_circ,'FaceAlpha',1);
xlabel('Excecution time [ms]')
ylabel('Occurance')
l=legend('Clothoidal LPT','Circular LPT','Location','N');
set(l,'FontSize',14);
set(gca,'FontSize',14)
hold off
if saveFigure; saveCurrentFigure('TimeBench_MultiObs_hist'); end

mean_dt_circ=mean(dt_circ);
mean_n_obs_circ = mean(n_obs_circ);
mean_dt_per_obs_circ = mean(dt_circ./n_obs_circ)*1000;

mean_dt_cloth=mean(dt_cloth);
mean_n_obs_cloth = mean(n_obs_cloth);
mean_dt_per_obs_cloth = mean(dt_cloth./n_obs_cloth)*1000;

fprintf('\nmean circular LPT time: %3.0f msec\n',mean_dt_circ)
fprintf('mean obstacles circular LPT: %3.0f\n',mean_n_obs_circ)
fprintf('mean circular LPT time / obs: %3.0f �sec\n\n',mean_dt_per_obs_circ)

fprintf('mean clothoidal LPT time: %3.0f msec\n',mean_dt_cloth)
fprintf('mean obstacles clothoidal LPT: %3.0f\n',mean_n_obs_cloth)
fprintf('mean clothoidal LPT time / obs: %3.0f �sec\n\n',mean_dt_per_obs_cloth)

fprintf('clothoidal LPT is %2.0f%% slower compared to the circular LPT\n',mean_dt_cloth/mean_dt_circ*100)
fprintf('clothoidal LPT has %2.0f%% more obstacles compared to the circular LPT\n',mean_n_obs_cloth/mean_n_obs_circ*100)
fprintf('clothoidal LPT is %2.0f%% time/obscle slower compared to the circular LPT\n\n',mean_dt_per_obs_cloth/mean_dt_per_obs_circ*100)

figure()
hold on
plot(n_obs_cloth,dt_cloth,'.');
plot(n_obs_circ,dt_circ,'.');
xlabel('Number of occupied grid cells')
ylabel('Excecution time [ms]')
l=legend('Clothoidal LPT','Circular LPT','Location','NW');
set(l,'FontSize',14);
set(gca,'FontSize',14)
hold off
if saveFigure; saveCurrentFigure('TimeBench_MultiObs_ObstacleInfluence'); end
end
%% Functions
function [dt_sum,n_obs]=AdjustPathLength(LSL,ObstacleTable,XY_ObsTable,XY_occ_lab_R)
tic
idxFound=find(ismember(XY_ObsTable,XY_occ_lab_R));
lenIdxFound = length(idxFound);
for ii=1:lenIdxFound
    idxRow=idxFound(ii);
    IDOccPaths = [ObstacleTable(idxRow).ID];
    IdxOccPaths = [ObstacleTable(idxRow).Idx];
    for jj=1:length(IDOccPaths)
        LSL(IDOccPaths(jj)).free=false;
        if LSL(IDOccPaths(jj)).idxBlocked > IdxOccPaths(jj)
            LSL(IDOccPaths(jj)).idxBlocked=IdxOccPaths(jj);
        end
    end
end
dt_sum = toc*1000;
n_obs=length(idxFound);
end

function [robotPose,XY_occ_lab]=getUseDefPose(selectMap)

switch selectMap
    case 1 % use RobotLab_Elevator map
        map = 'RobotLaboLiftEdges.bmp';
    case 2 % use RobotLab map
        map = 'RobotLaboEdges.bmp';
    case 3 % use RobotLab_ZoomEntrance map
        map = 'RobotLaboEntranceEdges.bmp';
    case 4 % use Elevator map
        map = 'LiftEdges.bmp';
    otherwise
        fprintf('Wrong input. Please use selectMap = {[1],2,3,4} to select {[RobotLab_Elevator], RobotLab, RobotLab_ZoomEntrance, Elevator} \n')
        fprintf('default value 1 chosen \n')
        map = 'RobotLaboLiftEdges.bmp';
end
[LabGrid,XY_occ_lab] = getMapXYOccFormBmp(map,0.02); 

figureFullScreen()
hold on
show(LabGrid)
title('Select begin position (1st click) and orientation (2nd click)')
set(gca,'FontSize',14)
set(gca, 'box', 'off')
[xStart,yStart]=ginput(1);
scatter(xStart,yStart,'*r')
[xOrient,yOrient]=ginput(1);
plot([xStart xOrient],[yStart yOrient],'*-r')
thStart=atan2(yOrient-yStart,xOrient-xStart);
robotPose=[xStart yStart thStart];
fprintf('Use defined robot pose [%2.2f, %2.2f, %2.2f�] \n', robotPose(1),robotPose(2),robotPose(3)*180/pi);
close all
end

function XY_occ_lab_R=getXY_occ_lab_R(XY_occ_lab,robotPose)

XY_occ_lab_R=TransRotXY(XY_occ_lab,-robotPose(3),-robotPose(1),-robotPose(2));
XY_occ_lab_R(abs(XY_occ_lab_R(:,1))>8,:)=[];
XY_occ_lab_R(abs(XY_occ_lab_R(:,2))>8,:)=[];

LabGrid_R = robotics.BinaryOccupancyGrid(16,16,50);
LabGrid_R.GridLocationInWorld=[-8 -8];
setOccupancy(LabGrid_R,XY_occ_lab_R,ones(size(XY_occ_lab_R,1),1))
inflate(LabGrid_R,0.005)
[ii_lab_R,jj_lab_R]=meshgrid(1:LabGrid_R.GridSize(1),1:LabGrid_R.GridSize(2));
ii_all_lab_R=ii_lab_R(:);
jj_all_lab_R=jj_lab_R(:);
occval_lab =getOccupancy(LabGrid_R,[ii_all_lab_R jj_all_lab_R], 'grid');
ii_occ_lab=ii_all_lab_R(occval_lab==1);
jj_occ_lab=jj_all_lab_R(occval_lab==1);
XY_occ_lab_R=grid2world(LabGrid_R,[ii_occ_lab jj_occ_lab]);
XY_occ_lab_R=unique(round(round(XY_occ_lab_R./0.02)*0.02,2),'rows');
XY_occ_lab_R=sortrows(XY_occ_lab_R);
end