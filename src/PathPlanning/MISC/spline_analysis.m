close all
clear
clc

[x,y]=ginput()

P=BezierCurve(x,y);
xx=P(:,1);
yy=P(:,2);

figure(1)
scatter(x,y); hold on
plot(xx,yy)