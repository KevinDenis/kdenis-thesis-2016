clear
close all
clc

DiffMat4=-eye(4,4)+diag([1 1 1],1); DiffMat4(end,:)=[];
DiffMat3=-eye(3,3)+diag([1 1],1); DiffMat3(end,:)=[];
% x = [x1 x2 x3 x4]';
% y = [y1 y2 y3 y4]';

t=0:0.05:1;



x1=0;
y1=0;
y2=y1;
x4=2;
y4=2;
x3=x4;
n = 4;
cvx_begin
    variables x(n,1) y(n,1)
    P=[x y];
    dP=[DiffMat4*x DiffMat4*y];
    ddP=[DiffMat3*DiffMat4*x DiffMat3*DiffMat4*x];
    B=bernsteinMatrix(3, t)*P;
    dB=n*bernsteinMatrix(2,t)*dP;
    ddB=(2)*(3)*bernsteinMatrix(1,t)*ddP;   
    dx=dB(:,1);dy=dB(:,2);ddx=ddB(:,1);ddy=ddB(:,2);

    minimize(max(ddx.^2)+max(ddy.^2)+max(dx.^2)+max(dy.^2))
    subject to
      x([1 3 4])==[x1 x4 x4]';
      y([1 2 4])==[y1 y2 y4]';
cvx_end

P=[x y]

[B,dB,ddB,kappa]=BezierCurve(P);

figure(1)
plot(B(:,1),B(:,2)); hold on
scatter(P(:,1),P(:,2)); hold off

figure(2)
plot(B(:,1),kappa)