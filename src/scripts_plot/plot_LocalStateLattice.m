initPlotScripts

load('grid_XY.mat')
load('LSLset.mat')
co=get(groot,'DefaultAxesColorOrder');

robotPose = [0 0 0];

plotClothCurve=true; % bézier : false,  cloth:true
if plotClothCurve
    load('LSL_cloth.mat')
    StringCurve='Clothoids';
    StringFileName='LSLClothoid';
    
else
    load('LSL_bezier.mat')
    StringCurve='Bézier Curve';
    StringFileName='LSLBezierCurve';
end

LSL=getForwardMotionFromStateLattice(LSL);
% [ReducedLSL] = ReducePathDensity(LSL,LSLset);

[MP] = getMotionPrimitiveFromStateLattice(LSL);

[LSL_red] =LSL;

fig=figureFullScreen();
fig.Renderer='Painters';
title('')
hold on
grid on
xlabel('x [m]')
ylabel('y [m]')
plotGrid(grid_XY,robotPose)
plotPath(LSL_red)
plotSimpleRobot(robotPose)
plotStateLatticePoints(MP,LSLset)
axis equal
l=legend('Discrete grids',StringCurve,'Reachable grids','Robot pose','Expention Position','Location','SW');
set(l,'FontSize',26);
set(gca,'FontSize',24)

axis([-2 4.1 -3.1 3.1])

% saveCurrentFigure(StringFileName);
