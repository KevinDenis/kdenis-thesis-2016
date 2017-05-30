function plotRoboticWheelchair(robotPose)
%PLOTSIMPLEROBOT Summary of this function goes here
%   Detailed explanation goes here

RobotHull=[-0.80  0.20;
           -0.46  0.30;
            0.50  0.30;
            0.50 -0.30;
           -0.46 -0.30;
           -0.80 -0.20;
           -0.80  0.20];
       
RobotHull=RotTransXY(RobotHull,robotPose(3),robotPose(1),robotPose(2));
plot(RobotHull(:,1),RobotHull(:,2),'k','Linewidth',1)
end
