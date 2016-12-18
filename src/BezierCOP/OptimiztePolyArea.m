clear
close all
clc

t=linspace(0,1,101);
x=[0 1 1 1]';
y=[0 0 2 2]';
k=convhull(x,y);
[B,dB,ddB,kappa]=BezierCurve(x,y,t);
tic
[bezierMatrix,zeroToZMatrix,zToTMatrix]=getBezierMatrix(length(x)-1,0.5);
toc
x_start=zeroToZMatrix*x;
y_start=zeroToZMatrix*y;
k_start=convhull(x_start,y_start);
x_end=zToTMatrix*x;
y_end=zToTMatrix*y;
k_end=convhull(x_end,y_end);


co=get(gca,'ColorOrder'); % get default color for plot
figure(1)
hold on
plot(B(:,1),B(:,2),'LineWidth',1.5); 
scatter(x,y,[],co(1,:),'filled')
fill(x(k),y(k),co(1,:),'FaceAlpha',0.2)

[B1,~,~,~]=BezierCurve(x_start,y_start,t);
plot(B1(:,1),B1(:,2),'Color',co(2,:),'LineWidth',1.5); 
scatter(x_start,y_start,[],co(2,:),'filled')
fill(x_start(k_start),y_start(k_start),co(2,:),'FaceAlpha',0.2)

[B2,~,~,~]=BezierCurve(x_end,y_end,t);
plot(B2(:,1),B2(:,2),'Color',co(3,:),'LineWidth',1.5); 
scatter(x_end,y_end,[],co(3,:),'filled')
fill(x_end(k_end),y_end(k_end),co(3,:),'FaceAlpha',0.2)

surfaceBoth=polyarea(x_end(k_end),y_end(k_end))+polyarea(x_start(k_start),y_start(k_start))
surface=polyarea(x(k),y(k))

% %% analysis
% subArea=cumtrapz(B(:,1),B(:,2));
% figure()
% plot(B(:,1),subArea)



function f=objFunction()

end

function g=cons()

end