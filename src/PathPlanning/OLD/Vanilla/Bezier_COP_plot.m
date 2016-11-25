clear
close all
clf
clc

r=0.01;

obs=[1.0 0.5;1.5 0.5;1.5 0.0;1.0 0.0];
if ~isempty(obs)
    obs_filled=[];
    for ii=1:length(obs)-1
        obs_filled=[obs_filled;fillline(obs(ii,:),obs(ii+1,:),40)];
    end
    obs_filled=[obs_filled;fillline(obs(end,:),obs(1,:),40)];
    obs=obs_filled;
end
 
 co=[0         0.4470    0.7410
     0.8500    0.3250    0.0980
     0.9290    0.6940    0.1250
     0.4940    0.1840    0.5560
     0.4660    0.6740    0.1880
     0.3010    0.7450    0.9330
     0.6350    0.0780    0.1840];
 
alpha=5;
n=6;
 
tic
[COP] = getCOP(alpha,n,r,obs);
z_sol = fmincon(COP);
x_sol=z_sol(1:2:end);
y_sol=z_sol(2:2:end);
toc

t=linspace(0,1,101)';
[B,dB,ddB,kappa]=BezierCurve([x_sol y_sol],t);
ds=cumtrapz(t,sqrt(dB(:,1).^2+dB(:,2).^2));

figure(1)
title('Bezier curve with control points')
hold on
plot(B(:,1),B(:,2),'color',co(1,:)); 
scatter(x_sol,y_sol,'filled','MarkerFaceColor',co(1,:));
if ~isempty(obs); fill(obs(:,1),obs(:,2),'k'); end
xlabel('x [m]')
ylabel('y [m]')


obs=[];
if ~isempty(obs)
    obs_filled=[];
    for ii=1:length(obs)-1
        obs_filled=[obs_filled;fillline(obs(ii,:),obs(ii+1,:),40)];
    end
    obs_filled=[obs_filled;fillline(obs(end,:),obs(1,:),40)];
    obs=obs_filled;
end
 
 co=[0         0.4470    0.7410
     0.8500    0.3250    0.0980
     0.9290    0.6940    0.1250
     0.4940    0.1840    0.5560
     0.4660    0.6740    0.1880
     0.3010    0.7450    0.9330
     0.6350    0.0780    0.1840];

tic
[COP] = getCOP(alpha,n,r,obs);
z_sol = fmincon(COP);
x_sol=z_sol(1:2:end);
y_sol=z_sol(2:2:end);
toc

t=linspace(0,1,101)';
[B,dB,ddB,kappa]=BezierCurve([x_sol y_sol],t);
ds=cumtrapz(t,sqrt(dB(:,1).^2+dB(:,2).^2));

title('Bezier curve with control points')
plot(B(:,1),B(:,2),'color',co(2,:)); 
scatter(x_sol,y_sol,'filled','MarkerFaceColor',co(2,:));
if ~isempty(obs); fill(obs(:,1),obs(:,2),'k'); end
xlabel('x [m]')
ylabel('y [m]')
hold off

legend('Bezier curve with obstacle','Bezier Control Point','Obstacle','Bezier curve without obstacle','Bezier Control Point')
