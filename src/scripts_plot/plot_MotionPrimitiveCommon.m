initWorkspace
co=get(groot,'DefaultAxesColorOrder');

robotPose = [0 0 0];

load('SL_bezier.mat')
SL_bezier=StateLattice;
SL_bezier=getMotionPrimitiveFromStateLattice(SL_bezier);
SL_bezier = CleanupStateLattice(SL_bezier);

load('SL_cloth.mat')
SL_cloth=StateLattice;
SL_cloth=getMotionPrimitiveFromStateLattice(SL_cloth);
SL_cloth = CleanupStateLattice(SL_cloth);


voxel_cloth=[[SL_cloth.x0].',[SL_cloth.y0].',[SL_cloth.th0].',[SL_cloth.x1].',[SL_cloth.y1].',[SL_cloth.th1].'];
voxel_bezier=[[SL_bezier.x0].',[SL_bezier.y0].',[SL_bezier.th0].',[SL_bezier.x1].',[SL_bezier.y1].',[SL_bezier.th1].'];

communCloth=ismember(voxel_cloth,voxel_bezier,'rows');

communBezier=ismember(voxel_bezier,voxel_cloth,'rows');



fig=figureFullScreen();
fig.Renderer='Painters';
title('')
hold on
grid on
xlabel('x [m]')
ylabel('y [m]')
plotGrid(grid_XY,robotPose)
plot(ROI0(:,1),ROI0(:,2),'-','Linewidth',2,'Color',co(2,:))
plot(grid_XY(idxIn0,1),grid_XY(idxIn0,2),'o','Color',co(2,:),'MarkerSize',10)
plotPath(SL_bezier(communBezier))
plotSimpleRobot(robotPose)
l=legend('Discrete grids','ROI','Grids In ROI','Common Bézier','Reachable grids','Robot pose','Location','SE');
set(l,'FontSize',26);
set(gca,'FontSize',24)
axis equal
axis([-1.5 3.5 -2 2])
saveCurrentFigure('MPBezierCurveCommon');




fig=figureFullScreen();
fig.Renderer='Painters';
title('')
hold on
grid on
xlabel('x [m]')
ylabel('y [m]')
plotGrid(grid_XY,robotPose)
plot(ROI0(:,1),ROI0(:,2),'-','Linewidth',2,'Color',co(2,:))
plot(grid_XY(idxIn0,1),grid_XY(idxIn0,2),'o','Color',co(2,:),'MarkerSize',10)
plotPath(SL_cloth(communCloth))
plotSimpleRobot(robotPose)
l=legend('Discrete grids','ROI','Grids In ROI','Common Clothoid','Reachable grids','Robot pose','Location','SE');
set(l,'FontSize',26);
set(gca,'FontSize',24)
axis equal
axis([-1.5 3.5 -2 2])
saveCurrentFigure('MPClothoidCommon');

