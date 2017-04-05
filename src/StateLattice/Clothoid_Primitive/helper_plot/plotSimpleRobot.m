function plotSimpleRobot(robotPose)
%plotSimpleRobot(robotPos)
%   Detailed explanation goes here
%#ok<*AGROW>
co=get(groot,'DefaultAxesColorOrder');

th_res=pi/32;
rRob=0.25;
rDir=0.4;
thRob=0:th_res:2*pi;

[m,~]=size(robotPose);

xRob=[];
yRob=[];

for ii=1:m

xRobCircle=robotPose(ii,1)+rRob*cos(thRob);
yRobCircle=robotPose(ii,2)+rRob*sin(thRob);

xRobDir=[robotPose(ii,1),robotPose(ii,1)+rDir*cos(robotPose(ii,3))];
yRobDir=[robotPose(ii,2),robotPose(ii,2)+rDir*sin(robotPose(ii,3))];

xRob=[xRob xRobCircle nan xRobDir nan];
yRob=[yRob yRobCircle nan yRobDir nan];

end

plot(xRob, yRob,'Color','k','LineWidth',2)
end