clear
close all
clc

x1=0; y1=0; th1=pi/4; Pstart=[x1,y1,th1]; % start sposition and orientation
xend=2; yend=2; thend=pi/2; Pend=[xend,yend,thend]; % end position and orientation
n=4; % order of the bezier curve
t=linspace(0,1,101)'; % overall resolution of the solver
kappa_max=1.5;
initCOP=struct('Pstart',Pstart,'Pend',Pend,'n',n,'t',t,'kappa_max',kappa_max);

u=1;
v=1;

[B,dB,ddB,kappa]=BezierCurve_FixedPose(Pstart,Pend,u,v,t);

plot(B(:,1),B(:,2)); hold on
x =[         0;
    0.7071;
    2.0000;
    2.0000];

y =[

         0;
    0.7071;
    1.0000;
    2.0000]; scatter(x,y)