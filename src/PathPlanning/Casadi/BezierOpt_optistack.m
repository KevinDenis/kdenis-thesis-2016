clear
close all
clf
clc
import casadi.*

n=4;
alpha=10;
counter =0;
alpha=20;

r=0.05;


% x = [x1 x2 x3 x4]';
% y = [y1 y2 y3 y4]';

t=linspace(0,1,101)';

x=optivar(n,1);
x.setInit(1:length(x))
y=optivar(n,1);
y.setInit(1:length(x))

tic
sol = optisolve(costFunction(x,y,alpha),constraints(x,y));
x_sol=optival(x);
y_sol=optival(y);
disp(' ')
disp(' ')
clc
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