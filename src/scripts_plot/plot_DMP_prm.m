
initPlotScripts
load('OptPath_prm.mat')

% show(map)
figure()

% show(mapInflated)

show(prm)
hold on

plotRobotPath(OptPath)


figure()
title('Local Path Planning')
hold on
hold off