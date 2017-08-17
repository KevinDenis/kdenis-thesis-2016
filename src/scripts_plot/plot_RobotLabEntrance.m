initPlotScripts

driveDir = 'fwd';
map = 'RobotLaboEntranceEdges.bmp';
load('RobotLaboEntrance_Corners_XY_CCW.mat')
Map_XY_CCW=RobotLaboEntrance_Corners_XY_CCW;
robotPose=[4.4 2.06 -pi];
goalIdx = [122 551];
if ~exist('LSLset', 'var'); load('LSLset.mat'); end
if ~exist('grid_XY', 'var'); load('grid_XY.mat'); end
if ~exist('LSL_cloth', 'var'); load('LSL_cloth.mat'); LSL_cloth = LSL; end
if ~exist('ObstacleTable_cloth', 'var'); load('ObstacleTable_cloth.mat'); ObstacleTable_cloth=ObstacleTable; end
if ~exist('XY_ObsTable_cloth', 'var'); load('XY_ObsTable_cloth.mat'); XY_ObsTable_cloth=XY_ObsTable; end
if ~exist('LSL_circ', 'var'); load('LSL_circ.mat'); LSL_circ = LSL; end
if ~exist('ObstacleTable_circ', 'var'); load('ObstacleTable_circ.mat'); ObstacleTable_circ=ObstacleTable; end
if ~exist('XY_ObsTable_circ', 'var'); load('XY_ObsTable_circ.mat'); XY_ObsTable_circ=XY_ObsTable; end

[LSL_cloth_W]=BuildLSLColFree(LSL_cloth,ObstacleTable_cloth,XY_ObsTable_cloth,[],map,robotPose,driveDir,0);
[LSL_circ_W]=BuildLSLColFree(LSL_circ,ObstacleTable_circ,XY_ObsTable_circ,[],map,robotPose,driveDir,0);

figureFullScreen(1)
hold on
plot(Map_XY_CCW(:,1),Map_XY_CCW(:,2),'k','LineWidth',3)
plotPath(LSL_cloth_W)
plotRoboticWheelchair(robotPose)
axis equal
axis([0 5.5 0 4]) 
hold off
xlabel('x [m]')
ylabel('y [m]')
% l=legend('Obstacle','Clothoid path','Robot pose','Location','SE');
% set(l,'FontSize',30);
set(gca,'FontSize',28)
saveCurrentFigure('EnterRobotLabCloth')

figureFullScreen(2)
hold on
plot(Map_XY_CCW(:,1),Map_XY_CCW(:,2),'k','LineWidth',3)
plotPath(LSL_circ_W)
plotRoboticWheelchair(robotPose)
axis equal
axis([0 5.5 0 4]) 
hold off
xlabel('x [m]')
ylabel('y [m]')
% l=legend('Obstacle','Circular path','Robot pose','Location','SE');
% set(l,'FontSize',30);
set(gca,'FontSize',28)
saveCurrentFigure('EnterRobotLabCirc');

figureFullScreen(3)
subplot(1,2,1)
hold on
plot(Map_XY_CCW(:,1),Map_XY_CCW(:,2),'k','LineWidth',3)
plotPath(LSL_circ_W)
plotRoboticWheelchair(robotPose)
axis equal
axis([0 5.5 0 4]) 
hold off
xlabel('x [m]')
ylabel('y [m]')
% l=legend('Obstacle','Circular path','Robot pose','Location','SE');
% set(l,'FontSize',30);
set(gca,'FontSize',28)
subplot(1,2,2)
hold on
plot(Map_XY_CCW(:,1),Map_XY_CCW(:,2),'k','LineWidth',3)
plotPath(LSL_cloth_W)
plotRoboticWheelchair(robotPose)
axis equal
axis([0 5.5 0 4]) 
hold off
xlabel('x [m]')
ylabel('y [m]')
% l=legend('Obstacle','Clothoid path','Robot pose','Location','SE');
% set(l,'FontSize',30);
set(gca,'FontSize',28)
saveCurrentFigure('EnterRobotLab_sum')
