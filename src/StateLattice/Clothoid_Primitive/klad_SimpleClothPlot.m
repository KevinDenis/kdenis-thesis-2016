clear
close all
clc

x0=0;
y0=0;
th0=-pi/4;
x1=1;
y1=1;
th1=0;

npts=100;

[kappa,dkappa,Ltot,iter] = buildClothoid(x0,y0,th0,x1,y1,th1);
s=linspace(0,Ltot,npts);
ks=kappa+dkappa*s;
intKappa=trapz(s,abs(ks));
[X,Y] = pointsOnClothoid(x0,y0,th0,kappa,dkappa,Ltot,npts);
TH=wrapToPi(cumtrapz(s,ks)+th0);

plot(s,TH)


