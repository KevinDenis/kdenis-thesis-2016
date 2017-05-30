function [XY_occ] = getOccXYFromBmpRobot(bmp,res)
%[XY_occ] = getOccXYFromBmpRobot(bmp,res)
RobotCxy=[0.8 0.3];
% Import Data from bitmap picture
RobotBW = ~imread(bmp);
robotGrid = robotics.BinaryOccupancyGrid(RobotBW,1/res);
robotGrid.GridLocationInWorld=-RobotCxy;
% Get all occupied XY positions
[ii,jj]=meshgrid(1:robotGrid.GridSize(1),1:robotGrid.GridSize(2));
ii_all=ii(:);
jj_all=jj(:);
occval =getOccupancy(robotGrid,[ii_all jj_all], 'grid');
ii_occ=ii_all(occval==1);
jj_occ=jj_all(occval==1);
XY_occ=grid2world(robotGrid,[ii_occ jj_occ]);
end

