clear
close all
clc

r=0.01;
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

alpha=0;
n=4;
 
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
% subplot(2,1,1)
title('Bezier curve with control points')
hold on
plot(B(:,1),B(:,2)); 
scatter(x_sol,y_sol,'filled');
if ~isempty(obs); fill(obs(:,1),obs(:,2),'k'); end
xlabel('x [m]')
ylabel('y [m]')
hold off