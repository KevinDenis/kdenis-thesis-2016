robotPose=[4.3918    2.0509    pi];
saveCurrentFigure('EnterRobotLabCloth')
close all
[LSL_W_circ,~]=BuildLSLFree(SL_circ,ObstacleTable_circ,XY_ObsTable_circ,grid_XY,'RobotLaboEntranceEdges.bmp',robotPose);
saveCurrentFigure('EnterRobotLabCirc')