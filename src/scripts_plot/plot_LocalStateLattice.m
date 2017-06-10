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

figureFullScreen()
title('')
hold on
grid on
xlabel('x [m]')
ylabel('y [m]')
plotGrid(grid_XY,robotPose)
plotPath(LSL_red)
plotRoboticWheelchair(robotPose)
plotStateLatticePoints(MP,LSLset)
plotReachableEndPoses(LSL_red,robotPose);

l=legend('Discrete grids',StringCurve,'Robot pose','Expansion Position','Reachable end pose','Location','SW');
set(l,'FontSize',26);
set(gca,'FontSize',24)
axis equal
axis([-2 4.1 -3.1 3.1])

saveCurrentFigure(StringFileName);
