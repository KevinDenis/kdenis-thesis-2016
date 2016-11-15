clear
close all
clc


Np = 50;
mi = 40/Np;
Di = 70*Np;
g = 9.81;
L=0;

x=optivar(Np,1);
y=optivar(Np,1);
sol = optisolve(potentialEnergy(x,y,mi,Di,g,L,Np),constraints(x,y));
x_sol=optival(x);
y_sol=optival(y);
plot(x_sol,y_sol,'-o'); hold on


L=1;
x=optivar(Np,1);
x.setInit(x_sol)
y=optivar(Np,1);
y.setInit(y_sol)
sol = optisolve(potentialEnergy(x,y,mi,Di,g,L,Np),constraints(x,y));
x=optival(x);
y=optival(y);
plot(x,y,'-o');

%{
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
    
    g{1}= x([1,end])==[x1 xend]';
    g{2}= y([1,end])==[y1 yend]';
    g{3}= y>=0.5;
end
%}