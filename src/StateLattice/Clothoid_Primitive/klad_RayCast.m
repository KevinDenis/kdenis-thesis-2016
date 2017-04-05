clear
close all
clc

% robot pose
robotPose=[-1 3 0];


dx=0.25;
dth=pi/8;
showPlot=false;

% Import Image
LabBW = ~imread('ComplicatedLab.bmp');
LabGrid = robotics.OccupancyGrid(LabBW,100);
LabGrid.GridLocationInWorld=[-5 -5];

figure()
hold on
show(LabGrid)
[xStart,yStart]=ginput(1);
scatter(xStart,yStart,'*r')
[xOrient,yOrient]=ginput(1);
plot([xStart xOrient],[yStart yOrient],'*-r')
thStart=atan2(yOrient-yStart,xOrient-xStart);
xStart=round(xStart/dx)*dx;
yStart=round(yStart/dx)*dx;
thStart=round(thStart/dth)*dth;


robotPose=[xStart yStart thStart];
angles=[0:1:360].*180./pi;
maxrange=5;
intsectionPts = rayIntersection(LabGrid,robotPose,angles,maxrange,0.7);
intsectionPts(isnan(intsectionPts(:,1)),:)=[];
scatter(intsectionPts(:,1),intsectionPts(:,2))

LabGridNew = robotics.BinaryOccupancyGrid(10,10,10);
LabGridNew.GridLocationInWorld=[-5 -5];
setOccupancy(LabGridNew,intsectionPts,ones(length(intsectionPts),1))
figure()
show(LabGridNew)