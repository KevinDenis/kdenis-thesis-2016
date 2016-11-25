clc

import casadi.*

x= MX.sym('x',4,1);
y= MX.sym('y',4,1);

diffMatrix(x)