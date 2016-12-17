clear 
close all
clc

x1=0; 
y1=0; 
th1=-pi/2; 
Pstart=[x1,y1,th1]; 
xend=2; 
yend=0; 
thend=pi/2; 
Pend=[xend,yend,thend];
n=4;
t=linspace(0,1,101)';

initCOP=struct('Pstart',Pstart,'Pend',Pend,'n',n,'t',t);

[x_init, y_init]=getInitVal(initCOP);

[B,dB,~,kappa]=BezierCurve(x_init,y_init,t);
title('State Latice for origin to [2 2]')
hold on
plot(B(:,1),B(:,2),'LineWidth',1.5); 
scatter(x_init,y_init,[],'filled')
axis equal
hold off
xlabel('x [m]')
ylabel('y [m]')
