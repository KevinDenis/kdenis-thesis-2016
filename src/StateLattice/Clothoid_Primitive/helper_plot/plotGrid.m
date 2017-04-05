function plotGrid(grid_XY,robotPose)
%plotSimpleRobot(robotPos)
%   Detailed explanation goes here
grid_XY_W=RotTransXY(grid_XY,robotPose(3),robotPose(1),robotPose(2));
plot(grid_XY_W(:,1),grid_XY_W(:,2),'o','Color',[0.5 .5 .5])
end