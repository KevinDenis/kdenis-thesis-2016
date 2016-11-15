function [C,Ceq]=nonlincon(x,y,t)
[~,~,~,kappa]=BezierCurve([x,y],t);
C=norm(kappa,inf)-2;
Ceq=[];
end
