clear
close all
clc
import casadi.*


P = MX.sym('x',4,2);
% P_val=[0,0;1,0;2,1;2,2];
% P(:,:)=P_val;
% [B,dB,ddB,kappa]=BezierCurve(P);


obs=[1.0 0.5;1.5 0.5;1.5 0.0;1.0 0.0];

[k,d] = dsearchnCasadi(obs,P)