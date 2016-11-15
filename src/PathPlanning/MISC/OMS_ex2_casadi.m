clear
close all
clc
import casadi.*


Np = 50;
mi = 40/Np;
Di = 70*Np;
g = 9.81;
L=0;

x=MX.sym('x',Np,1); y=MX.sym('y',Np,1);
nlp = struct('x', [ x;y] , 'f',potentialEnergy(x,y,mi,Di,g,L,Np),'g',constraints(x,y));
S=nlpsol('S','ipopt',nlp);

r=S('x0', [ones(50,1);ones(50,1)] , 'lbg',0 , 'ubg' , 0 );

x_opt=full(r.x(1:Np));
y_opt=full(r.x(Np+1:end));

display(x_opt);


plot(x_opt,y_opt,'-o'); hold on



function V=potentialEnergy(x,y,mi,Di,g,L,Np)
V=0;

for ii=1:length(x)-1
    dx=x(ii)-x(ii+1);
    dy=y(ii)-y(ii+1);
    V=V+0.5*Di*(sqrt(dx^2+dy^2)-L/Np)^2;
end

for ii=1:length(y)
    V=V+mi*g*y(ii);
end

end

function g=constraints(x,y)
    x1=-2;
    y1=1;
    xend=-x1;
    yend=y1;

%     g{1}= x([1,end])==[x1 xend]';
%     g{2}= y([1,end])==[y1 yend]';
%     g{3}= y>=0.5;
    
    g{1}=x(1)-x1;
    g{2}=x(end)- xend;
    g{3}=y(1)-y1;
    g{4}=y(end)-yend;
    g{5}=y>=0.5;
    disp(g)
    g = vertcat(g{:});  
    disp(g)
end