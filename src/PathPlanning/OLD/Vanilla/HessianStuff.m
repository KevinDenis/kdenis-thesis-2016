clear 
close all
clc

syms x1 x2 x3 x4
syms y1 y2 y3 y4
syms t

x=[x1 x2 x3 x4].';
y=[y1 y2 y3 y4].';

n=length(x)-1;

P=[x,y];
dP=[diffMatrix(x) diffMatrix(y)];
ddP=[diffMatrix(diffMatrix(x)) diffMatrix(diffMatrix(y))];
B=bernsteinMatrix(n, t)*P; pretty(B)
dB=n*bernsteinMatrix(n-1,t)*dP; pretty(dB)
ddB=(n-1)*(n)*bernsteinMatrix(n-2,t)*ddP; pretty(ddB)

dx=dB(:,1);
dy=dB(:,2);
ddx=ddB(:,1);
ddy=ddB(:,2);

kappa_mod=((dx.*ddy-dy.*ddx));

hessian(kappa_mod,[x;y])

x=[0 0.5 0.5 1]';
y=[0 0 1 1]';

[B,dB,ddB,kappa]=BezierCurve([x y]);

% plot(B(:,1),B(:,2))


dx=dB(:,1);
dy=dB(:,2);
ddx=ddB(:,1);
ddy=ddB(:,2);
kappa_mod=((dx.*ddy-dy.*ddx));

figure()
plot([kappa kappa_mod])

figure()
plot([dx dy ddx ddy])