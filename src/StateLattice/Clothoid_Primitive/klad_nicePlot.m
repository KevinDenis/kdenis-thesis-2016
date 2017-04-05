% figure(1)
% hold on
% show(PathOccGrid)
% plot(X(1:kk),Y(1:kk),'r','LineWidth',3)
% hold off
% grid on

RobotFullBW = ~imread('RobotFull.bmp');
robotFullGrid = robotics.BinaryOccupancyGrid(RobotFullBW,200);
RobotFullInfo=regionprops(RobotFullBW);
RobotFullCij=RobotFullInfo.Centroid;
RobotFullCxy=grid2world(robotFullGrid,round([RobotFullCij(2),RobotFullCij(1)]));
robotFullGrid.GridLocationInWorld=-RobotFullCxy;




figure(1)
show(robotFullGrid);
axis equal


