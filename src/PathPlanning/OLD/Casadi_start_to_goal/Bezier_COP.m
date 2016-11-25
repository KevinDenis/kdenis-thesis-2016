clear
close all
clf
clc
import casadi.*

n=4;
alpha=80;

r=0.05;

x1=0; y1=0; theta1=0; Pstart=[x1,y1,theta1];
xend=2; yend=2; thetaend=pi/2; Pend=[xend,yend,thetaend];


t=linspace(0,1,1001)';

x=optivar(n,1);
x.setInit(1:n)
y=optivar(n,1);
x.setInit(1:n)
tic
sol = optisolve(costFunction(x,y,alpha),constraints(x,y));
x_sol=optival(x);
y_sol=optival(y);
disp(costFunction(x_sol,y_sol,alpha))
toc

[B,~,~,kappa]=BezierCurve([x_sol y_sol],t);

figure(1)
subplot(2,1,1)
title('Bezier curve with control points')
hold on
plot(B(:,1),B(:,2)); 
xlabel('x [m]')
ylabel('y [m]')
axis equal

subplot(2,1,2)
title('Curvature \kappa(t)')
hold on;
plot(t,kappa)
xlabel('t [/]')
ylabel('\kappa [1/m]')
hold off