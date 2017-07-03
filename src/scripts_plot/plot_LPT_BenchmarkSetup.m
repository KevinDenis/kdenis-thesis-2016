initPlotScripts

load('Benchmark1_fine.mat')
map = 'RobotLaboEntranceEdges.bmp';
load('RobotLaboEntrance_Corners_XY_CCW.mat')
Map_XY_CCW=RobotLaboEntrance_Corners_XY_CCW;
SimpleMapXY = Map_XY_CCW;
SimpleMapXY(9,1)=8;
SimpleMapXY(10,:)=[];
SimpleMapXY(10,1)=8;
SimpleMapXY=[SimpleMapXY;[0.01,0.01;2.11,0.01]];

RobotHull= [-0.80,  0.20;
            -0.46,  0.30;
             0.50,  0.30;
             0.50, -0.30;
            -0.46, -0.30;
            -0.80, -0.20;
            -0.80,  0.20];
RobotHull=LinInterpToRes(RobotHull,0.05);

angle_range = pi/4;
angle_res=pi/8;
TH = -angle_range:angle_res:angle_range;
grid_XY=[3.5 2.2; 4 1; 4 3];
grid_XY=sortrows(grid_XY);
startEnd_X = [grid_XY(:,1), repelem(GoalTarget(1),length(grid_XY(:,1)),1),nan(length(grid_XY(:,1)),1)].';
startEnd_Y = [grid_XY(:,2), repelem(GoalTarget(2),length(grid_XY(:,2)),1),nan(length(grid_XY(:,2)),1)].';
startEnd = [startEnd_X(:), startEnd_Y(:)];

grid_XYTH_tmp=addTHtoGridXY(grid_XY,TH.');
grid_XYTH = grid_XYTH_tmp;
idxVec=1:length(TH);
lenVec=length(idxVec);
for ii=1:size(grid_XY,1)
    kk=(ii-1)*lenVec+idxVec;
    dP = GoalTarget-grid_XY(ii,:);
    angle_shift = atan2(dP(2),dP(1));
    TH_new = wrap2Pi(round((TH+angle_shift)/angle_res)*angle_res);
    grid_XYTH(kk,3)=TH_new;
end

dP_startEnd = GoalTarget - grid_XY(1,:);
angleCircle = atan2(dP_startEnd(2),dP_startEnd(1));

[x_circleShift,y_circleShift]=XY_Circle(grid_XY(1,:),0,angleCircle,0.55);
[x_circleRange,y_circleRange]=XY_Circle(grid_XY(1,:),grid_XYTH(1,3),wrapTo2Pi(grid_XYTH(5,3)),0.40);
[x_circleRes,y_circleRes]=XY_Circle(grid_XY(1,:),grid_XYTH(1,3),wrapTo2Pi(grid_XYTH(2,3)),0.25);

toKeep=false(length(grid_XYTH),1);
for ii = 1:length(grid_XYTH)
    robotPose = grid_XYTH(ii,:);
    RobotHull_RotTrans = RotTransXY(RobotHull,robotPose(3),robotPose(1),robotPose(2));
    if all(InPolygon(RobotHull_RotTrans(:,1),RobotHull_RotTrans(:,2),SimpleMapXY(:,1),SimpleMapXY(:,2)))
        toKeep(ii)=1;
    end
end
grid_XYTH_keep=grid_XYTH(toKeep,:);

exampleDelete=find(~toKeep);

figureFullScreen(2)
hold on
% plotPath(LSL_cloth_W)
plot(TestRegion(:,1),TestRegion(:,2),'r-','LineWidth',2)
plot(GoalRegion(:,1),GoalRegion(:,2),'g-','LineWidth',2)
plot(GoalTarget(:,1),GoalTarget(:,2),'g*','LineWidth',2,'MarkerSize',20)
plotRobotPose(grid_XYTH(toKeep,:),'k')
plotRobotPose(grid_XYTH(~toKeep,:),':k')

[grid_XY]=polygrid(TestRegion(:,1),TestRegion(:,2),(1/0.1)^2);
plot(grid_XY(:,1),grid_XY(:,2),'k.')
plot(startEnd(:,1),startEnd(:,2),'k--')
plot(x_circleRange,y_circleRange,'-.k','LineWidth',1.5)
text(3.25,2.65,'$\theta_{range}$','Interpreter','LaTeX','FontSize',30)
plot(x_circleRes,y_circleRes,'-.k','LineWidth',1.5)
text(3.45,2.45,'$\theta_{res}$','Interpreter','LaTeX','FontSize',30)
plot(Map_XY_CCW(:,1),Map_XY_CCW(:,2),'k','LineWidth',3)
axis equal
axis([1 5.5 0 4]) 
hold off
xlabel('x [m]')
ylabel('y [m]')
l=legend('Test region','Goal region','Target','Selected start pose','Removed robot start pose','Start Position','Location','SW');
set(l,'FontSize',30);
set(gca,'FontSize',28)

saveCurrentFigure('BenchmarkSetup')

[grid_XY]=polygrid(TestRegion(:,1),TestRegion(:,2),(1/0.1)^2);
grid_XYTH_tmp=addTHtoGridXY(grid_XY,TH.');
grid_XYTH = grid_XYTH_tmp;
idxVec=1:length(TH);
lenVec=length(idxVec);
for ii=1:size(grid_XY,1)
    kk=(ii-1)*lenVec+idxVec;
    dP = GoalTarget-grid_XY(ii,:);
    angle_shift = atan2(dP(2),dP(1));
    TH_new = wrap2Pi(round((TH+angle_shift)/angle_res)*angle_res);
    grid_XYTH(kk,3)=TH_new;
end
toKeep=false(length(grid_XYTH),1);
for ii = 1:length(grid_XYTH)
    robotPose = grid_XYTH(ii,:);
    RobotHull_RotTrans = RotTransXY(RobotHull,robotPose(3),robotPose(1),robotPose(2));
    if all(InPolygon(RobotHull_RotTrans(:,1),RobotHull_RotTrans(:,2),SimpleMapXY(:,1),SimpleMapXY(:,2)))
        toKeep(ii)=1;
    end
end
grid_XYTH=grid_XYTH(toKeep,:);

figureFullScreen(1)
hold on
% plotPath(LSL_cloth_W)
plot(TestRegion(:,1),TestRegion(:,2),'r-','LineWidth',2)
plot(GoalRegion(:,1),GoalRegion(:,2),'g-','LineWidth',2)
plot(GoalTarget(:,1),GoalTarget(:,2),'g*','LineWidth',2,'MarkerSize',20)
plotRobotPose2(grid_XYTH,'k')
plot(Map_XY_CCW(:,1),Map_XY_CCW(:,2),'k','LineWidth',3)
axis equal
axis([1 5.5 0 4]) 
hold off
xlabel('x [m]')
ylabel('y [m]')
l=legend('Test region','Goal region','Target','Selected start pose','Location','SW');
set(l,'FontSize',30);
set(gca,'FontSize',28)

saveCurrentFigure('BenchmarkSetupAll')

%% FUNCTIONS
function grid_X_Y_TH=addTHtoGridXY(grid_X_Y,TH)
%grid_X_Y_TH=addTHtoGridXY(grid_X_Y,TH)
%   Adds end positions pased on TH to grid_X_Y
grid_X_Y_rep=repmat(grid_X_Y,length(TH),1);
TH_rep=repelem(TH,length(grid_X_Y));
grid_X_Y_TH=[grid_X_Y_rep TH_rep];
grid_X_Y_TH=unique(grid_X_Y_TH,'rows');
end


function plotRobotPose(grid_XYTH,plotColor)
r=0.2;
X_Pose=[grid_XYTH(:,1), grid_XYTH(:,1)+r*cos(grid_XYTH(:,3)), nan(size(grid_XYTH(:,1)))].';
Y_Pose=[grid_XYTH(:,2), grid_XYTH(:,2)+r*sin(grid_XYTH(:,3)), nan(size(grid_XYTH(:,1)))].';
plot(X_Pose(:).',Y_Pose(:).',plotColor,'LineWidth',1.5)
end

function plotRobotPose2(grid_XYTH,plotColor)
r=0.05;
X_Pose=[grid_XYTH(:,1), grid_XYTH(:,1)+r*cos(grid_XYTH(:,3)), nan(size(grid_XYTH(:,1)))].';
Y_Pose=[grid_XYTH(:,2), grid_XYTH(:,2)+r*sin(grid_XYTH(:,3)), nan(size(grid_XYTH(:,1)))].';
plot(X_Pose(:).',Y_Pose(:).',plotColor,'LineWidth',1.5)
end


function [x_circle,y_circle]=XY_Circle(XY_c,th_start,th_end,r_c)
x_circle = r_c * cos([th_start:pi/32:th_end th_end])+XY_c(1);
y_circle = r_c * sin([th_start:pi/32:th_end th_end])+XY_c(2);
end