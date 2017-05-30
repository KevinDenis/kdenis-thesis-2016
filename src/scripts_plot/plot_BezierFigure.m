initWorkspace

co=get(groot,'DefaultAxesColorOrder');

t=linspace(0,1,1001);
P1 = [ 5 -10];
P2 = [18 18];
P3 = [38 -5];
P4 = [45 15];
Pn=[P1; P2; P3; P4];

B=BezierCurve(Pn(:,1),Pn(:,2),t) ;
kk=convhull(Pn(:,1),Pn(:,2));

figure()
xlim([0 50])
axis equal
hold on
fill(Pn(kk,1),Pn(kk,2),co(2,:),'FaceAlpha',0.2)
plot(Pn(:,1),Pn(:,2),'-o','Color',co(2,:),'Linewidth',2)
plot(B(:,1),B(:,2),'Color',co(1,:),'Linewidth',2)
placelabel(P1,'P_1');
placelabel(P2,'P_2');
placelabel(P3,'P_3');
placelabel(P4,'P_4');
hold off
l=legend('Control Polygon','Control Points','B�zier Curve','Location','SE');
set(l,'FontSize',12);


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
axis([-0.1 1.1 -0.1 1.1])
axis equal
hold on
plot(BC(:,1),BC(:,2),'k--','Linewidth',0.5)
plot(B(:,1),B(:,2),'Color',co(1,:),'Linewidth',2)
plot(Pn(:,1),Pn(:,2),'o','Color',co(2,:),'MarkerFaceColor',co(2,:),'Linewidth',2)
placelabel(P1,'P_1');
placelabel(P2,'P_2');
placelabel(P3,'P_3');
placelabel(P4,'P_4');
hold off
l=legend('Constraints for P_2 and P_3','B�zier Curve','Control Points','Location','SE');
set(l,'FontSize',12);
% pause()
% saveCurrentFigure

%{
a = -3*t.^2 +  6*t - 3;
b =  9*t.^2 - 12*t + 3;
c = -9*t.^2 +  6*t;
d =  3*t.^2;

tvec = kron(a,pt1) + kron(b,pt2) + kron(c,pt3) + kron(d,pt4);

%%
% This gives us the tangent vector at any point on the curve. We can these
% to our plot with hold on, but we won't draw all of them because it gets
% too crowded.
for i=1:10:101
    l = line([pts(1,i), pts(1,i)+tvec(1,i)/6], ... 
             [pts(2,i), pts(2,i)+tvec(2,i)/6]);
    l.Color = 'green';
end

%%
% If you look at the values of those derivatives, you'll see an interesting
% pattern. When $t$ equals 0, the four terms are [-3, 3, 0, 0]. That means 
% that the tangent of the start of the curve is simply $3pt_2 - 3pt_1$. The
% same thing happens at the other end when $t$ equals 1. 
%
% This makes it easy to connect two Bézier curves so that they tangent at 
% the meeting point. You just make the last two points of the first curve 
% colinear with the first two points of the second curve.
%
cla
xlim([0 50])
axis equal

pt1 = [ 5;-10];
pt2 = [13; -2];
pt3 = [ 5; 20];
pt4 = [25; 20];

placelabel(pt1,'pt_1');
placelabel(pt2,'pt_2');
placelabel(pt3,'pt_3');
placelabel(pt4,'pt_4');

pt5 = [45; 20];
pt6 = [35; -2];
pt7 = [43;-10];

placelabel(pt5,'pt_5');
placelabel(pt6,'pt_6');
placelabel(pt7,'pt_7');

%%
% Notice that $pt3$, $pt4$, and $pt5$ are colinear:
pt4 - pt3
pt5 - pt4

%%
% And if we plot the curves, we can see that we get a nice, smooth join 
% where they meet.

pts1 = kron((1-t).^3,pt1) + kron(3*(1-t).^2.*t,pt2) + kron(3*(1-t).*t.^2,pt3) + kron(t.^3,pt4);
pts2 = kron((1-t).^3,pt4) + kron(3*(1-t).^2.*t,pt5) + kron(3*(1-t).*t.^2,pt6) + kron(t.^3,pt7);

hold on
plot(pts1(1,:),pts1(2,:))
plot(pts2(1,:),pts2(2,:))
hold off

%%
% You can find a list of other useful properties of the Bézier curve at this 
% <http://en.wikipedia.org/wiki/B%C3%A9zier_curve#Properties Wikipedia
% page>.
%
% The teapot is actually made of Bézier patches. These are parametric surfaces 
% which are made from the same math as Bézier curves, but with two
% parameters (usually called $u$ and $v$) instead of the one parameter (i.e. $t$)
% that we've been using here. We'll come back to investigate Bézier patches
% in a future post, but first we're going to explore how to draw some other
% useful curves.

%%
% _Copyright 2014 The MathWorks, Inc._
%}

function placelabel(pt,str)
    x = pt(1);
    y = pt(2);
    h = line(x,y);
    h = text(x,y,str,'FontSize',12);
    h.HorizontalAlignment = 'center';
    h.VerticalAlignment = 'bottom';
end