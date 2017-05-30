initWorkspace
load('OptPath_voronoi.mat')

figure()


path= VoronoiOptimalPath(startLocation, endLocation);


plotRobotPath(OptPath)


figure()
title('Local Path Planning')
hold on
hold off