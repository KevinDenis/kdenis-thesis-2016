initPlotScripts


[FresnelC,FresnelS] = FresnelCS(-5:0.0001:5);

figure(1)
plot(FresnelC,FresnelS,'LineWidth',1)
axis(0.75*[-1 1 -1 1])
axis equal
set(gca, 'box', 'off')
saveCurrentFigure('Design_GenClothoid')