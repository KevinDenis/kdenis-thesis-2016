initWorkspace
%% Init workspace for benchmark. 
% Will load data unless it is allready in workspace
if ~exist('LSLset', 'var'); load('LSLset.mat'); end
if ~exist('grid_XY', 'var'); load('grid_XY.mat'); end
if ~exist('LSL_cloth', 'var'); load('LSL_cloth.mat'); LSL_cloth = LSL; end
if ~exist('ObstacleTable_cloth', 'var'); load('ObstacleTable_cloth.mat'); ObstacleTable_cloth=ObstacleTable; end
if ~exist('XY_ObsTable_cloth', 'var'); load('XY_ObsTable_cloth.mat'); XY_ObsTable_cloth=XY_ObsTable; end
if ~exist('LSL_circ', 'var'); load('LSL_circ.mat'); LSL_circ = LSL; end
if ~exist('ObstacleTable_circ', 'var'); load('ObstacleTable_circ.mat'); ObstacleTable_circ=ObstacleTable; end
if ~exist('XY_ObsTable_circ', 'var'); load('XY_ObsTable_circ.mat'); XY_ObsTable_circ=XY_ObsTable; end

% Additional Data
map = 'LiftEdges.bmp';
load('LiftEntrance_Corners_XY_CCW.mat')
Map_XY_CCW=LiftEntrance_Corners_XY_CCW;

SimpleLabCoordinates=[0,    5.67;
                      4.77, 5.67;
                      4.77, 3.99;
                      8.00, 3.99;
                      8.00, 7.39;
                      0,    7.39;
                      0,    5.67;];

RobotHull=[-0.80  0.20;
           -0.46  0.30;
            0.50  0.30;
            0.50 -0.30;
           -0.46 -0.30;
           -0.80 -0.20;
           -0.80  0.20];
TestRegion = [
              4.5, 5.8;
              6.0, 5.8;
              6.0, 7.3;
              4.5, 7.3;
              4.5, 5.8;
              ];

GoalRegion = [
              6.8, 4.4;
              7.2, 4.4;
              7.2, 5.2;
              6.8, 5.2;
              6.8, 4.4;
              ];
GoalRegion=flip(GoalRegion);
RobotHull=LinInterpToRes(RobotHull,0.05);
% Normal Test Grid
[grid_XY]=polygrid(TestRegion(:,1),TestRegion(:,2),400);
TH = wrap2Pi(pi/2:pi/16:3/2*pi);
% % Fine Test Grid
% [grid_XY]=polygrid(TestRegion(:,1),TestRegion(:,2),1600);
% TH = wrap2Pi(pi/2:pi/32:3/2*pi);
grid_XYTH=addTHtoGridXY(grid_XY,TH);
% [LabGrid,XY_occ] = getMapXYOccFormBmp(map,0.02);

toKeep=false(length(grid_XYTH),1);
for ii = 1:length(grid_XYTH)
    robotPose = grid_XYTH(ii,:);
    RobotHull_RotTrans = RotTransXY(RobotHull,robotPose(3),robotPose(1),robotPose(2));
    if all(InPolygon(RobotHull_RotTrans(:,1),RobotHull_RotTrans(:,2),SimpleLabCoordinates(:,1),SimpleLabCoordinates(:,2)))
        toKeep(ii)=1;
    end
end


grid_XYTH=grid_XYTH(toKeep,:);
figure()
hold on
plot(Map_XY_CCW(:,1),Map_XY_CCW(:,2),'k-','LineWidth',2)
plot(TestRegion(:,1),TestRegion(:,2),'r-')
plot(GoalRegion(:,1),GoalRegion(:,2),'g-')
plotRobotPose(grid_XYTH,'k')

pathInGoal_cloth=false(length(grid_XYTH),1);
pathInGoal_circ=false(length(grid_XYTH),1);
n=length(grid_XYTH);
% ppm = ParforProgressStarter2('Calculating...', n, 0.1, 0, 1, 1);
for ii=1:n
% 1700
    [LSL_W_cloth,~,~]=BuildLSLColFree(LSL_cloth,ObstacleTable_cloth,XY_ObsTable_cloth,grid_XY,map,grid_XYTH(ii,:),'bkw');
    [LSL_W_circ,~,~]=BuildLSLColFree(LSL_circ,ObstacleTable_circ,XY_ObsTable_circ,grid_XY,map,grid_XYTH(ii,:),'bkw');    
    pathInGoal_cloth(ii)=pathThroughGoalRegion(GoalRegion,LSL_W_cloth);
% % If visual inspection is desired
%     if pathInGoal_cloth(ii)
%         clf
%         figure(1)
%         hold on
%         plot(Map_XY_CCW(:,1),Map_XY_CCW(:,2),'k-','LineWidth',2)
%         plot(TestRegion(:,1),TestRegion(:,2),'r-')
%         plot(GoalRegion(:,1),GoalRegion(:,2),'g-')
%         plotRobotPose(grid_XYTH,'k')
%         plotRobotPath(LSL_W_cloth)
%         hold off
%         drawnow
%         disp(ii)
%     end
    pathInGoal_circ(ii)=pathThroughGoalRegion(GoalRegion,LSL_W_circ);
%     ppm.increment(ii);
end
% delete(ppm);

% plotRobotPose(grid_XYTH(pathInGoal_circ,:),'r')
% plotRobotPose(grid_XYTH(pathInGoal_cloth,:),'b')

% save('Benchmark2_medium.mat')

%% FUNCTIONS

function pathInGoal=pathThroughGoalRegion(GoalRegion,Path)
for ii=1:length(Path)
    if ~Path(ii).free
        idxPlot=Path(ii).idxBlocked;
        Path(ii).X=Path(ii).X(1:idxPlot-1);
        Path(ii).Y=Path(ii).Y(1:idxPlot-1);
        Path(ii).TH=Path(ii).TH(1:idxPlot-1);
    end
end

ii_todelete=zeros(1);
nn=1;
for ii=1:length(Path)
    if isempty(Path(ii).X)
        ii_todelete(nn)=ii;
        nn=nn+1;
    end
end
if nn ~= 1
    Path(ii_todelete)=[];
end

X_path=zeros(1);
Y_path=zeros(1);
kk=0;
for ii=1:length(Path)
    kk=kk(end)+(1:(length(Path(ii).X)+1));
    X_path(kk)=[Path(ii).X;nan];
    Y_path(kk)=[Path(ii).Y;nan];
end

if any(InPolygon(X_path,Y_path,GoalRegion(:,1),GoalRegion(:,2)))
    pathInGoal=true;
else
    pathInGoal = false;
end
end

%}
function plotRobotPose(grid_XYTH,plotColor)
r=0.01;
X_Pose=[grid_XYTH(:,1), grid_XYTH(:,1)+r*cos(grid_XYTH(:,3)), nan(size(grid_XYTH(:,1)))].';
Y_Pose=[grid_XYTH(:,2), grid_XYTH(:,2)+r*sin(grid_XYTH(:,3)), nan(size(grid_XYTH(:,1)))].';
plot(X_Pose(:).',Y_Pose(:).',plotColor,'LineWidth',1.5)
end

function grid_X_Y_TH=addTHtoGridXY(grid_X_Y,TH)
%grid_X_Y_TH=addTHtoGridXY(grid_X_Y,TH)
%   Adds end positions pased on TH to grid_X_Y
grid_X_Y_rep=repmat(grid_X_Y,length(TH),1);
TH_rep=repelem(TH,length(grid_X_Y)).';
grid_X_Y_TH=[grid_X_Y_rep TH_rep];
grid_X_Y_TH=unique(grid_X_Y_TH,'rows');
end