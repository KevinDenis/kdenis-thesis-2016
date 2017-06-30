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

selectMap = 4; % RobotLab_Elevator=1, RobotLab=2, RobotLab_ZoomEntrance=3, Elevator=4

[userRobotPose,XY_occ_lab]=getUseDefPose(selectMap);

n_des = 1000;
n_space = round(n_des^(1/3));
n_angle = round(n_des/n_space^2);
x_var=linspace(-0.25,0.25,n_space);
y_var=x_var;
th_var=linspace(-pi/2,pi/2,n_angle);

robotPose_vec = getAllComb(x_var,y_var,th_var);

allRobotPoses = repmat(userRobotPose,size(robotPose_vec,1),1)+robotPose_vec;

n=size(robotPose_vec,1);
delMaxdT=floor(n/10);
dt_circ=zeros(n,1);
n_obs_circ=zeros(n,1);
dt_cloth=zeros(n,1);
n_obs_cloth=zeros(n,1);
ppm = ParforProgressStarter2('Calculating...', n, 0.1, 0, 1, 1);
parfor ii=1:n
    XY_occ_lab_R=getXY_occ_lab_R(XY_occ_lab,allRobotPoses(ii,:));
    [dt_circ_ii,n_obs_circ_ii]=AdjustPathLength(LSL_circ,ObstacleTable_circ,XY_ObsTable_circ,XY_occ_lab_R);
    [dt_cloth_ii,n_obs_cloth_ii]=AdjustPathLength(LSL_cloth,ObstacleTable_cloth,XY_ObsTable_cloth,XY_occ_lab_R);
    dt_circ(ii)=dt_circ_ii;
    n_obs_circ(ii)=n_obs_circ_ii;
    dt_cloth(ii)=dt_cloth_ii;
    n_obs_cloth(ii)=n_obs_cloth_ii;
    ppm.increment(ii);
end
delete(ppm);
[dt_cloth,idx_cloth_S]=sort(dt_cloth);
n_obs_cloth=n_obs_cloth(idx_cloth_S);
dt_cloth(end-delMaxdT:end)=[];
n_obs_cloth(end-delMaxdT:end)=[];


[dt_circ,idx_circ_S]=sort(dt_circ);
n_obs_circ=n_obs_circ(idx_circ_S);
dt_circ(end-10:end)=[];
n_obs_circ(end-10:end)=[];

figure()
hold on
histogram(dt_cloth);
histogram(dt_circ);

figure()
hold on
plot(n_obs_cloth,dt_cloth,'.');
plot(n_obs_circ,dt_circ,'.');

figure()
hold on
histogram(dt_cloth./n_obs_cloth);
histogram(dt_circ./n_obs_circ);

%% Functions
function [dt,n_obs]=AdjustPathLength(LSL,ObstacleTable,XY_ObsTable,XY_occ_lab_R)
tic
idxFound=find(ismember(XY_ObsTable,XY_occ_lab_R,'rows'));
for ii=1:length(idxFound)
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
dt=toc*1000;
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
robotPose=[5.24, 6.05, -76.29*pi/180] ;
fprintf('Use defined robot pose [%2.2f, %2.2f, %2.2f°] \n', robotPose(1),robotPose(2),robotPose(3)*180/pi);
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