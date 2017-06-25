initPlotScripts

co=get(groot,'DefaultAxesColorOrder');

t=linspace(0,1,1001);
P1 = [ 5 -10]/45;
dP1= [-0.05 0];
P2 = [20 15]/45;
dP2= [0 0.01];
P3 = [38 -5]/45;
dP3= [0.075 -0.025];
P4 = [45 15]/45;
dP4= [0.01 0.01];
Pn=[P1; P2; P3; P4];

B=BezierCurve(Pn(:,1),Pn(:,2),t) ;
kk=convhull(Pn(:,1),Pn(:,2));

figure()
set(gcf,'renderer','painters');
xlim([-0.05 1.05])
ylim([-0.25 0.4])
% axis equal
hold on
fill(Pn(kk,1),Pn(kk,2),co(2,:),'FaceAlpha',0.2)
plot(Pn(:,1),Pn(:,2),'-o','Color',co(2,:),'Linewidth',2)
plot(B(:,1),B(:,2),'Color',co(1,:),'Linewidth',2)
placelabel(P1+dP1,'P_1');
placelabel(P2+dP2,'P_2');
placelabel(P3+dP3,'P_3');
placelabel(P4+dP4,'P_4');
xlabel('x [m]')
ylabel('y [m]')
hold off
l=legend('Convex hull','Control Points','Bézier Curve','Location','NW');
set(l,'FontSize',12);

% saveCurrentFigure('MP_BezierExample')

P1 = [ 0  0];
P2 = [0.5 0];
P3 = [0.5 0.5];
P4 = [1 1];

Pn=[P1; P2; P3; P4];
B=BezierCurve(Pn(:,1),Pn(:,2),t);

BC1=[P1;1.1 0];
BC2=[-0.3 -0.3;P4];
BC=[BC1;nan nan;BC2];

figure()
axis([-0.1 1.1 -0.2 1.1])
axis equal
hold on
plot(BC(:,1),BC(:,2),'k--','Linewidth',0.5)
plot(B(:,1),B(:,2),'Color',co(1,:),'Linewidth',2)
plot(Pn(:,1),Pn(:,2),'o','Color',co(2,:),'MarkerFaceColor',co(2,:),'Linewidth',2)
placelabel(P1,'P_1');
placelabel(P2,'P_2');
placelabel(P3,'P_3');
placelabel(P4,'P_4');
xlabel('x [m]')
ylabel('y [m]')
hold off
l=legend('Constraints for P_2 and P_3','Bézier Curve','Control Points','Location','NW');
set(l,'FontSize',12);

% pause()
saveCurrentFigure('MP_BezierCubicConstraints')
%}
function placelabel(pt,str)
    x = pt(1);
    y = pt(2);
    h = line(x,y);
    h = text(x,y,str,'FontSize',12);
    h.HorizontalAlignment = 'center';
    h.VerticalAlignment = 'bottom';
end
