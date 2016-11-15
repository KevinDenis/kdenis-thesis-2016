clear
close all
clc

t = linspace(0,1,2)';

P1=[0 0];
P2=[1 1];
P3=[1 2];
P4=[2 2];
P5=[2 0];
P=[P1;P2;P3;P4;P5];

P=P(1:4,:);



figure(1)
[B,dB,ddB] = BezierCurve(P,t);
plot(B(:,1),ddB(:,1)); hold on

[B,dB,ddB] = BezierCurve3(P,t);
plot(B(:,1),ddB(:,1));