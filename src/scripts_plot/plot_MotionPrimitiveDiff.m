initWorkspace
load('grid_XY.mat')
co=get(groot,'DefaultAxesColorOrder');
%#ok<*UNRCH>

robotPose = [0 0 0];

load('SL_bezier.mat')
SL_bezier=StateLattice;
SL_bezier=getMotionPrimitiveFromStateLattice(SL_bezier);
SL_bezier = CleanupStateLattice(SL_bezier);
load('SL_cloth.mat')
SL_cloth=StateLattice;
SL_cloth=getForwardMotionFromStateLattice(SL_cloth);
SL_cloth=getMotionPrimitiveFromStateLattice(SL_cloth);
SL_cloth = CleanupStateLattice(SL_cloth);


voxel_cloth=[[SL_cloth.x0].',[SL_cloth.y0].',[SL_cloth.th0].',[SL_cloth.x1].',[SL_cloth.y1].',[SL_cloth.th1].'];
voxel_bezier=[[SL_bezier.x0].',[SL_bezier.y0].',[SL_bezier.th0].',[SL_bezier.x1].',[SL_bezier.y1].',[SL_bezier.th1].'];

uniqueCloth=~ismember(voxel_cloth,voxel_bezier,'rows');
uniqueBezier=~ismember(voxel_bezier,voxel_cloth,'rows');

fig=figureFullScreen();
fig.Renderer='Painters';
title('')
hold on
grid on
xlabel('x [m]')
ylabel('y [m]')
plotGrid(grid_XY,robotPose)
plotPath(SL_bezier(uniqueBezier))
plotPath(SL_cloth(uniqueCloth),co(2,:),3)
plotSimpleRobot(robotPose)
l=legend('Discrete grids','Unique Bézier','Unique Clothoid','Robot pose','Location','SE');
set(l,'FontSize',26);
set(gca,'FontSize',24)
axis equal
axis([-1.1 3.1 -1.6 1.6])
saveCurrentFigure('MPDifference');