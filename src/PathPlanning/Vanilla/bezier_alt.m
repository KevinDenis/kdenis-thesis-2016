clear 
close all
clc 
t=linspace(0,1)';
  
P1=[0 0];
P2=[1 1];
P3=[1 2];
P4=[2 2];
P=[P1;P2;P3;P4];

Px=P(:,1);
Py=P(:,2);

[B,dB, ddB] = BezierCurveCubic(Px,Py);



BezierBasis=[-1, 3,-3, 1; 
              3,-6, 3, 0;
             -3, 3, 0, 0;
              1, 0, 0, 0];
        
tic
PP=[polyval(BezierBasis*Px, t) polyval(BezierBasis*Py, t)];
toc
figure()
plot(PP(:,1),PP(:,2)); hold on;
plot(B(:,1),B(:,2)); hold on;

polyder(BezierBasis*Px)

dPPx=polyval(polyder(BezierBasis*Px),t);
dPPy=polyval(polyder(BezierBasis*Py),t);
dPPy1=dPPy./dPPx;
dPPy2=dB(:,2)./dB(:,1);


[B,dB]=BezierCurve(P);
dB_dydx=dB(:,2)./dB(:,1);


plot(B(:,1),dB_dydx)

figure()
plot(PP(:,1),dPPy1); hold on
plot(PP(:,1),dPPy2);