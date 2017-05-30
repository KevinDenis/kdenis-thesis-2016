function [x_init, y_init,min_val,max_val]=getInitVal(initCOP)

n=initCOP.n;
P0=initCOP.P0;
P1=initCOP.P1;
x0=P0(1); y0=P0(2); th0=P0(3);
x1=P1(1); y1=P1(2); th1=P1(3);

max_val=max([x0 y0 x1 y1])*2;
min_val=-max_val;

r=0.25*max([abs(x1-x0),abs(y1-y0)]);

x_init=linspace(x0,x1,n)';
y_init=linspace(y0,y1,n)';

x_init(2)=x0+r*cos(th0);
y_init(2)=y0+r*sin(th0);


x_init(end-1)=x1-r*cos(th1);
y_init(end-1)=y1-r*sin(th1);

if ~any(diff(y_init))
    y_init(2)=y_init(1)+0.01;
    y_init(3)=y_init(1)-0.01;
end
end