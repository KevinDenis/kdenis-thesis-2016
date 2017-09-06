initPlotScripts
co=get(groot,'DefaultAxesColorOrder');

robotPose = [0 0 0];

load('LSL_bezier.mat')
LSL_bezier=LSL;
LSL_bezier=getMotionPrimitiveFromStateLattice(LSL_bezier);
LSL_bezier = CleanupLSL(LSL_bezier);

load('LSL_cloth.mat')
LSL_cloth=LSL;
LSL_cloth=getMotionPrimitiveFromStateLattice(LSL_cloth);
LSL_cloth = CleanupLSL(LSL_cloth);


vertices_cloth=getStartEndVerticesPath(LSL_cloth);
vertices_bezier=getStartEndVerticesPath(LSL_bezier);

communCloth=ismember(vertices_cloth,vertices_bezier,'rows');

communBezier=ismember(vertices_bezier,vertices_cloth,'rows');



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
plotPath(LSL_bezier(communBezier))
plotSimpleRobot(robotPose)
l=legend('Discrete grids','ROI','Grids In ROI','Common Bézier','Reachable grids','Robot pose','Location','SE');
set(l,'FontSize',26);
set(gca,'FontSize',24)
axis equal
axis([-1.5 3.5 -2 2])
% saveCurrentFigure('MPBezierCurveCommon');




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
plotPath(LSL_cloth(communCloth))
plotSimpleRobot(robotPose)
l=legend('Discrete grids','ROI','Grids In ROI','Common Clothoid','Reachable grids','Robot pose','Location','SE');
set(l,'FontSize',26);
set(gca,'FontSize',24)
axis equal
axis([-1.5 3.5 -2 2])
% saveCurrentFigure('MPClothoidCommon');

