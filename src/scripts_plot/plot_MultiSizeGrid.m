initPlotScripts
close all
co=get(groot,'DefaultAxesColorOrder');

[LSLset] = getLocalStateLatticeSettings();

figureFullScreen()
title('')
xlabel('x [m]')
ylabel('y [m]')
hold on
axis equal
plotGrid(grid_XY,[0 0 0])
plot(ROI0(:,1),ROI0(:,2),'-','Linewidth',2,'Color',co(2,:))
plot(grid_XY(idxIn0,1),grid_XY(idxIn0,2),'o','Color',co(2,:),'MarkerSize',10)
plotRoboticWheelchair([0 0 0])
l=legend('Discrete grids','ROI','Grids In ROI','Robot pose','Location','SE');
set(l,'FontSize',30);
set(gca,'FontSize',28)

axis([-4.5 4.5 -3.5 3.5])

saveCurrentFigure('MultiGirdWithROI');