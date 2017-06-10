initPlotScripts
co=get(groot,'DefaultAxesColorOrder');

%#ok<*UNRCH>

robotPose = [0 0 0];

plotClothCurve=true; % b�zier : false,  cloth:true
[LSLset] = getLocalStateLatticeSettings();
[grid_XY,ROI0,idxIn0,~,~]=BuildMultiSizeGrid(LSLset);

if plotClothCurve
    load('LSL_cloth.mat')
    StringCurve='Clothoid';
    StringFileName='MPClothoid';
    
else
    load('LSL_bezier.mat')
    StringCurve='B�zier Curve';
    StringFileName='MPBezierCurve';
end

MP=getMotionPrimitiveFromStateLattice(LSL);
MP=getForwardMotionFromStateLattice(MP);

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
l=legend('Discrete grids','ROI','Grids In ROI',StringCurve,'Reachable end pose','Robot pose','Location','SE');
set(l,'FontSize',26);
set(gca,'FontSize',24)
axis equal
axis([-1.5 3.5 -2 2])

saveCurrentFigure(StringFileName)