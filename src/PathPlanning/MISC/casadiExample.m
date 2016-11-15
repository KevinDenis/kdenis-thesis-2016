clear 
close all
clc
import casadi.*






%% NLP
x=SX.sym('x');
y=SX.sym('y');

nlp=struct('x',[x;y],'f',costFunction(x,y),'g',constrainsts(x,y));

S=nlpsol('S','ipopt',nlp);

r=S('x0', [0,0] , 'lbg',0 , 'ubg' , 0 );

x_opt=r.x;
display(x_opt);

%% QP
% x = SX.sym('x');
% y = SX.sym('y');
% qp = struct('x', [x;y], 'f',costFunction(x,y),'g', [x+y-10;x-1]);
% S =qpsol('S','qpoases', qp);
% r = S('lbg',0);
% x_opt = r.x;
% display(x_opt)

function f=costFunction(x,y)
    f=x^2+y^2
end

function g=constrainsts(x,y)
    g={(1-x)^2-y,x^2+y^2}
end