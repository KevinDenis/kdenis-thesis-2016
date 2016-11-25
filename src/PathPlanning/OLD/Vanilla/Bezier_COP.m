clear
close all
clc
% z(1,1)=if_else((x <= obs(1)),MX(1),MX(2))


x1=0;
y1=0;
theta1=0;
Pstart=[x1,y1,theta1];

xg=2;
yg=2;
thetag=pi/4;
Pgoal=[xg,yg,thetag];

co=get(gca,'ColorOrder');

obs=[];
% obs=[1.0 0.5;1.5 0.5;1.5 0.0;1.0 0.0];
if ~isempty(obs)
    obs_filled=[];
    for ii=1:length(obs)-1
        obs_filled=[obs_filled;fillline(obs(ii,:),obs(ii+1,:),40)];
    end
    obs_filled=[obs_filled;fillline(obs(end,:),obs(1,:),40)];
    obs=obs_filled;
end

alpha=10;
n=4;

for ii=1:1
tic
[COP] = getCOP(alpha,n,obs,Pstart,Pgoal);
z_sol = fmincon(COP);
x_sol=z_sol(1:2:end);
y_sol=z_sol(2:2:end);
x_init=COP.x0(1:2:end);
y_init=COP.x0(2:2:end);
disp(' ')
disp('=======================')
disp(' ')
toc
t=linspace(0,1,101)';
[B,dB,ddB,kappa]=BezierCurve([x_sol y_sol],t);
disp(Pgoal(1:2)-B(end,:))
disp(abs(thetag-atan(dB(end,2)/dB(end,1))))

figure(1)
subplot(2,1,1)
title('Bezier curve with control points')
hold on
plot(B(:,1),B(:,2),'Color',co(ii,:));
scatter(x_init,y_init,[],co(3,:),'filled');
scatter(x_sol,y_sol,[],co(ii,:),'filled');
if ~isempty(obs); fill(obs(:,1),obs(:,2),'k'); end
xlabel('x [m]')
ylabel('y [m]')
axis equal

subplot(2,1,2)
title('Curvature \kappa(t)')
hold on;
plot(t,kappa)
xlabel('t [/]')
ylabel('\kappa [1/m]')
hold off

x1=x_sol(end);
y1=y_sol(end);
theta1=atan(dB(end,2)/dB(end,1));
Pstart=[x1,y1,theta1];
n=4;
end