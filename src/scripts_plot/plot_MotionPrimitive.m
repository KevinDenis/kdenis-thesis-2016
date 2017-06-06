initPlotScripts
co=get(groot,'DefaultAxesColorOrder');

%#ok<*UNRCH>

robotPose = [0 0 0];

plotClothCurve=false; % bézier : false,  cloth:true
[LSLset] = getLocalStateLatticeSettings();
[grid_XY,ROI0,idxIn0,~,~]=BuildMultiSizeGrid(LSLset);

if plotClothCurve
    load('LSL_cloth.mat')
    StringCurve='Clothoids';
    StringFileName='MPClothoid';
    
else
    load('LSL_bezier.mat')
    StringCurve='Bézier Curve';
    StringFileName='MPBezierCurve';
end

MP=getMotionPrimitiveFromStateLattice(LSL);
MP=getForwardMotionFromStateLattice(MP);

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
plotPath(MP)
plotSimpleRobot(robotPose)
l=legend('Discrete grids','ROI','Grids In ROI',StringCurve,'Reachable grids','Robot pose','Location','SE');
set(l,'FontSize',26);
set(gca,'FontSize',24)
axis equal
axis([-1.5 3.5 -2 2])

% saveCurrentFigure(StringFileName);

%% Plot
% if showPlot  
%     
%     figure()  
%     plot(grid_XY(idxIn1,1),grid_XY(idxIn1,2),'ro')
%     plotSimpleRobot([1.5, 1, 1.1781])
%     l=legend('Discrete grids','ROI','Cells In ROI','Robot pose','Location','SE');
%     set(l,'FontSize',16);
%     set(gca,'FontSize',14)
%     axis([-7 8.5 -7.5 7.5])
%     pause()
%     saveCurrentFigure



%{

%% Plot
if showPlot
%     %% Motion Primitive 
%     figure()
%     title('')
%     hold on
%     grid on
%     xlabel('x [m]')
%     ylabel('y [m]')
%     axis equal
%     plotGrid(grid_XY,robotPose)
%     plot(ROI0(:,1),ROI0(:,2),'g-','Linewidth',2)
%     plot(grid_XY(idxIn0,1),grid_XY(idxIn0,2),'og')
%     plotPath(MotionPrem)
%     plotSimpleRobot(robotPose)
%     l=legend('Discrete grids','ROI','Grids In ROI','Clothoids','Circular arcs, straight lines','Reachable grids','Robot pose','Location','SE');
%     set(l,'FontSize',16);
%     set(gca,'FontSize',14)
%     axis([-1 4 -3 3])
%     pause()
%     saveCurrentFigure   
% 
%     %% Example of State Lattice Point [1.5 0 5*pi/8]
%     figure()
%     title('')
%     hold on
%     grid on
%     xlabel('x [m]')
%     ylabel('y [m]')
%     axis equal
%     plotGrid(grid_XY,robotPose)
%     plot(ROI1(:,1),ROI1(:,2),'r-','Linewidth',2)
%     plot(grid_XY(idxIn1,1),grid_XY(idxIn1,2),'ro')
%     plotPath([MotionPrem;MotionPremKeep])
%     plotSimpleRobot([1.5 1 1.1781])
%     l=legend('Discrete grids','ROI','Grids In ROI','Clothoids','Circular arcs, straight lines','Reachable grids','Robot pose','Location','SE');
%     set(l,'FontSize',16);
%     set(gca,'FontSize',14)
%     axis([-1 6 -3 4])
%     pause()
%     saveCurrentFigure   
%     
    %% Complete Set Of Paths Of State Lattice
    figure()
    title('')
    hold on
    grid on
    xlabel('x [m]')
    ylabel('y [m]')
    axis equal
    plotGrid(grid_XY,robotPose)
    plotPath(StateLattice)
    plotSimpleRobot(robotPose)
    plotStateLatticePoints(MotionPrem,LSLset)
    l=legend('Discrete grids','Clothoids','Circular arcs, straight lines','Reachable grids','Robot pose','State Lattice Positions','Location','SE');
    set(l,'FontSize',16);
    set(gca,'FontSize',14)
%     axis([-1 6 -4 4])
%     pause()
%     saveCurrentFigure
%
%}