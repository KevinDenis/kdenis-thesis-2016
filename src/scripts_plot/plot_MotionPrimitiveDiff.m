initPlotScripts
load('grid_XY.mat')
co=get(groot,'DefaultAxesColorOrder');
%#ok<*UNRCH>

robotPose = [0 0 0];

load('LSL_bezier.mat')
LSL_bezier=LSL;
LSL_bezier=getMotionPrimitiveFromStateLattice(LSL_bezier);
LSL_bezier = CleanupLSL(LSL_bezier);
load('LSL_cloth.mat')
LSL_cloth=LSL;
LSL_cloth=getForwardMotionFromStateLattice(LSL_cloth);
LSL_cloth=getMotionPrimitiveFromStateLattice(LSL_cloth);
LSL_cloth = CleanupLSL(LSL_cloth);


vertices_cloth=getStartEndVerticesPath(LSL_cloth);
vertices_bezier=getStartEndVerticesPath(LSL_bezier);

uniqueCloth=~ismember(vertices_cloth,vertices_bezier,'rows');
uniqueBezier=~ismember(vertices_bezier,vertices_cloth,'rows');

fig=figureFullScreen();
fig.Renderer='Painters';
title('')
hold on
grid on
xlabel('x [m]')
ylabel('y [m]')
plotGrid(grid_XY,robotPose)
plotPath(LSL_bezier(uniqueBezier))
plotPath(LSL_cloth(uniqueCloth),co(2,:),3)
plotRoboticWheelchair([0 0 0])
l=legend('Discrete grids','Unique Bézier','Unique Clothoid','Robot pose','Location','SE');
set(l,'FontSize',26);
set(gca,'FontSize',24)
axis equal
axis([-1.1 3.1 -1.6 1.6])
saveCurrentFigure('MPDifference');