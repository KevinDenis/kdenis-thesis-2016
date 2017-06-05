initWorkspace
close all
co=get(groot,'DefaultAxesColorOrder');

load('grid_XY.mat')
load('ROI0.mat')
load('idxIn0.mat')
load('LSLset.mat')
load('LSL_cloth.mat')


[LSL] = getForwardMotionFromStateLattice(LSL);
[MP] = getMotionPrimitiveFromStateLattice(LSL);

vertices_MP = getStartEndVerticesPath(MP);
vertices_LSL=getStartEndVerticesPath(LSL);


idxEPExample=findrow_mex(vertices_MP(:,1:5),[0 0 0 1.5 1]); % find example idx

endPose=vertices_MP(idxEPExample,4:6);

idxEPExample_post=ismember(vertices_LSL(:,1:3),endPose,'rows');

% idxEPAll = rem(norm(grid_XY(:,1)

ROI1=RotTransXY(ROI0,endPose(3),endPose(1),endPose(2));
idxIn1=InPolygon(grid_XY(:,1),grid_XY(:,2),ROI1(:,1),ROI1(:,2));

figureFullScreen()
% figure()
title('')
xlabel('x [m]')
ylabel('y [m]')
hold on
axis equal
plotGrid(grid_XY,[0 0 0])
plotStateLatticePoints(MP,LSLset)
plot(ROI1(:,1),ROI1(:,2),'-','Linewidth',2,'Color',co(2,:))
plot(grid_XY(idxIn1,1),grid_XY(idxIn1,2),'o','Color',co(2,:),'MarkerSize',10)
plotPath(MP(idxEPExample),co(1,:),3)
plotPath(LSL(idxEPExample_post),co(1,:),1)
plotSimpleRobot(endPose)
l=legend('Discrete grids','Expantion Position','ROI at EP','Grids In ROI','Clothoid leading to EP','Clothoids originating from EP','Reachable grids','Robot pose at EP','Location','SW');
set(l,'FontSize',30);
set(gca,'FontSize',28)

axis([-2.0 3.67 -0.05 3.49])

saveCurrentFigure('MultiGirdWithEPROI');
