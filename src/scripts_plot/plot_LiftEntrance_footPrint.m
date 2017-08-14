initPlotScripts

driveDir = 'bkw';
map = 'LiftEdges.bmp';
load('LiftEntrance_Corners_XY_CCW.mat')
Map_XY_CCW=LiftEntrance_Corners_XY_CCW;
robotPose_cloth=[4.25 6.2 pi];
robotPose_circ=[5.5 6.25 pi/2];
goalIdx_cloth = [137 722];
goalIdx_circ = [118];
if ~exist('LSLset', 'var'); load('LSLset.mat'); end
if ~exist('grid_XY', 'var'); load('grid_XY.mat'); end
if ~exist('LSL_cloth', 'var'); load('LSL_cloth.mat'); LSL_cloth = LSL; end
if ~exist('ObstacleTable_cloth', 'var'); load('ObstacleTable_cloth.mat'); ObstacleTable_cloth=ObstacleTable; end
if ~exist('XY_ObsTable_cloth', 'var'); load('XY_ObsTable_cloth.mat'); XY_ObsTable_cloth=XY_ObsTable; end
if ~exist('LSL_circ', 'var'); load('LSL_circ.mat'); LSL_circ = LSL; end
if ~exist('ObstacleTable_circ', 'var'); load('ObstacleTable_circ.mat'); ObstacleTable_circ=ObstacleTable; end
if ~exist('XY_ObsTable_circ', 'var'); load('XY_ObsTable_circ.mat'); XY_ObsTable_circ=XY_ObsTable; end

LSL_cloth=FreeAllPaths(LSL_cloth);

[LSL_cloth_W,robotPose_cloth,LabGrid]=BuildLSLColFree(LSL_cloth,ObstacleTable_cloth,XY_ObsTable_cloth,[],map,robotPose_cloth,driveDir,0);
[LSL_circ_W]=BuildLSLColFree(LSL_circ,ObstacleTable_circ,XY_ObsTable_circ,[],map,robotPose_circ,driveDir,0);

figureFullScreen(1)
hold on
plot(Map_XY_CCW(:,1),Map_XY_CCW(:,2),'k','LineWidth',3)
plotPath(LSL_cloth_W)
plotRobotPath(LSL_cloth_W(goalIdx_cloth))
plotRoboticWheelchair(robotPose_cloth)
axis equal
axis([3.5 8.25 3.95 7.5]) 
hold off
xlabel('x [m]')
ylabel('y [m]')
l=legend('Obstacle','Clothoidal LPT','Desired path','Robot footprint','Start pose','Location','SW');
set(l,'FontSize',30);
set(gca,'FontSize',28)
saveCurrentFigure('EnterLiftCloth_Footprint');

figureFullScreen(2)
hold on
plot(Map_XY_CCW(:,1),Map_XY_CCW(:,2),'k','LineWidth',3)
plotPath(LSL_circ_W)
plotRobotPath(LSL_circ_W(goalIdx_circ))
plotRoboticWheelchair(robotPose_circ)
axis equal
axis([3.5 8.25 3.95 7.5]) 
hold off
xlabel('x [m]')
ylabel('y [m]')
l=legend('Obstacle','Circular LPT','Desired path','Robot footprint','Start pose','Location','SW');
set(l,'FontSize',30);
set(gca,'FontSize',28)
saveCurrentFigure('EnterLiftCirc_Footprint');