clear variables
close all
clc

%{
%   costum : spline=csape(x,[y1 y yend],[i j]) different conditions at endpoints
%   knot-a-not = cte 2nd derivative
%   natural vs variational ?
%   periodic = same 2nd derivative at begin & end
%   complete : clamped --> spline(x,[y1' y yend'])
%}

n=4;
[x,y] = ginput(n);
y=y';

% x= [0.2362    0.3560    0.6164    0.7615];
% y=[ 0.1181    0.7157    0.2959    0.6720];

% cs = spline(x,[0 y 0]);
xx = linspace(min(x),max(x),1001);
h=xx(2)-xx(1);

% spline1=csapi(x,y);
spline1=csape(x,y,'not-a-knot');
% spline1=spline(x,[5 y 5]);
yy1=ppval(spline1,xx);
d_yy1=diff(yy1,1)/h;
dd_yy1=diff(yy1,2)/h^2;
% spline=spline(x,y')
spline2=csaps(x,y,1 - 9.5533e-05,[],[1 0.5 0.5 1]);
yy2=ppval(spline2,xx);
d_yy2=diff(yy2,1)/h;
dd_yy2=diff(yy2,2)/h^2;

P=BezierCurve(x,y);
xx3=P(:,1);
yy3=P(:,2);
h=xx3(2)-xx3(1);
d_yy3=diff(yy3,1)/h;
dd_yy3=diff(yy3,2)/h^2;

% figure(1)
% subplot(3,3,1)
% plot(x,y,'o',xx,yy1,'-');
% subplot(3,3,4)
% plot(xx(1:length(d_yy1)),d_yy1)
% subplot(3,3,7)
% plot(xx(1:length(dd_yy1)),dd_yy1)
% 
% subplot(3,3,2)
% plot(x,y,'o',xx,yy2,'-');
% subplot(3,3,5)
% plot(xx(1:length(d_yy2)),d_yy2)
% subplot(3,3,8)
% plot(xx(1:length(dd_yy2)),dd_yy2)
% 
% subplot(3,3,3)
% plot(x,y,'o',xx3,yy3,'-');
% subplot(3,3,6)
% plot(xx3(1:length(d_yy3)),d_yy3)
% subplot(3,3,9)
% plot(xx3(1:length(dd_yy3)),dd_yy3)

figure(1)
subplot(4,1,1)
plot(x,y,'o',xx,yy1,'-');
subplot(4,1,2)
plot(xx(1:length(d_yy1)),d_yy1)
subplot(4,1,3)
plot(xx(1:length(dd_yy1)),dd_yy1)
subplot(4,1,4)
plot(xx,radiusCurvature(xx,yy1))


