function [x_init, y_init]=getInitVal(initCOP)

n=initCOP.n;
Pstart=initCOP.Pstart;
Pend=initCOP.Pend;
x1=Pstart(1); y1=Pstart(2); theta1=Pstart(3);
xg=Pend(1);yg=Pend(2); thetag=Pend(3);


r=0.25*max([abs(xg-x1),abs(yg-y1)]);



x_init=linspace(x1,xg,n)';
y_init=linspace(y1,yg,n)';

x_init(2)=x1+r*cos(theta1);
y_init(2)=y1+r*sin(theta1);


x_init(end-1)=xg-r*cos(thetag);
y_init(end-1)=yg-r*sin(thetag);


end