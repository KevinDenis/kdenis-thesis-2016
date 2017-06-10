initWorkspace
co=get(groot,'DefaultAxesColorOrder');

load('LSL_cloth.mat')
MP=getMotionPrimitiveFromStateLattice(LSL);
MP=getForwardMotionFromStateLattice(MP);
[LSLset] = getLocalStateLatticeSettings();
[grid_XY,ROI0,idxIn0,~,~]=BuildMultiSizeGrid(LSLset);
figureFullScreen()
title('')
hold on
grid on
xlabel('x [m]')
ylabel('y [m]')
plotGrid(grid_XY,robotPose)
plot(ROI0(:,1),ROI0(:,2),'-','Linewidth',2,'Color',co(2,:))
plot(grid_XY(idxIn0,1),grid_XY(idxIn0,2),'o','Color',co(2,:),'MarkerSize',10)
plotPath(MP)
plotReachableEndPoses(MP,robotPose)
plotRoboticWheelchair(robotPose)
l=legend('Discrete grids','ROI','Grids In ROI','Clothoid','Reachable end pose','Robot pose','Location','SE');
set(l,'FontSize',26);
set(gca,'FontSize',24)
axis equal

saveCurrentFigure('test')









