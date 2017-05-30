initWorkspace
load('grid_XY.mat')
load('LSLset.mat')
co=get(groot,'DefaultAxesColorOrder');

robotPose = [0 0 0];

plotClothCurve=true; % bézier : false,  cloth:true
if plotClothCurve
    load('SL_cloth.mat')
    StringCurve='Clothoids';
    StringFileName='LSLClothoid';
    
else
    load('SL_bezier.mat')
    StringCurve='Bézier Curve';
    StringFileName='LSLBezierCurve';
end

% StateLattice=getForwardMotionFromStateLattice(StateLattice);
% [ReducedStateLattice] = ReducePathDensity(StateLattice,LSLset);

[MotionPrimitive] = getMotionPrimitiveFromStateLattice(StateLattice);

[ReducedStateLattice] =StateLattice;

voxel_cloth=[[StateLattice.x0].',[StateLattice.y0].',[StateLattice.th0].',[StateLattice.x1].',[StateLattice.y1].',[StateLattice.th1].'];

[~,idxUnique,~] = unique(voxel_cloth(:,4:6),'rows');

[ReducedStateLattice] =ReducedStateLattice(idxUnique);

% voxel_cloth=[[SL_cloth.x0].',[SL_cloth.y0].',[SL_cloth.th0].',[SL_cloth.x1].',[SL_cloth.y1].',[SL_cloth.th1].'];

fig=figureFullScreen();
fig.Renderer='Painters';
title('')
hold on
grid on
xlabel('x [m]')
ylabel('y [m]')
plotGrid(grid_XY,robotPose)

plotPath(ReducedStateLattice)
plotSimpleRobot(robotPose)
plotStateLatticePoints(MotionPrimitive,LSLset)
l=legend('Discrete grids',StringCurve,'Reachable grids','Robot pose','Expention Position','Location','SW');
set(l,'FontSize',26);
set(gca,'FontSize',24)
axis equal
axis([-2 4.1 -3.1 3.1])
% cleanfigure
% saveCurrentFigure(StringFileName);