clear
close all
clf
clc
import casadi.*

n=4;
alpha=10;
counter =0;
alpha=20;
co=[0         0.4470    0.7410
    0.8500    0.3250    0.0980
    0.9290    0.6940    0.1250
    0.4940    0.1840    0.5560
    0.4660    0.6740    0.1880
    0.3010    0.7450    0.9330
    0.6350    0.0780    0.1840];

obs=[0.5 0.5;
     1.0 0.5;
     1.0 1.0;
     0.5 1.0];
r=0.05;
obs_filled=[];
for ii=1:length(obs)-1
    obs_filled=[obs_filled;fillline(obs(ii,:),obs(ii+1,:),10)];
end
obs_filled=[obs_filled;fillline(obs(end,:),obs(1,:),10)];
obs=obs_filled;

n_mat=3:6;
alpha_mat=[0 1 5 20];
alpha_mat=20;
leg=cell(length(n_mat),1);
for alpha=alpha_mat
% for n=n_mat


for k=1:length(alpha_mat)
  leg{k}=sprintf('alpha = %d',alpha_mat(k));
end

% for k=1:length(n_mat)
%   leg{k}=sprintf('n= %d',n_mat(k));
% end

% for alpha=alpha


% x = [x1 x2 x3 x4]';
% y = [y1 y2 y3 y4]';

if counter < 7 ; counter=counter+1; else; counter=1; end % for the color
t=linspace(0,1,101)';

x=optivar(n,1);
x.setInit(1:length(x))
y=optivar(n,1);
y.setInit(1:length(x))

tic
sol = optisolve(costFunction(x,y,alpha),constraints(x,y,obs));
x_sol=optival(x);
y_sol=optival(y);
disp(' ')
disp(' ')
clc
disp(costFunction(x_sol,y_sol,alpha))
toc

[B,~,~,kappa]=BezierCurve([x_sol y_sol],t);


figure(1)
% subplot(2,1,1)
title('Bezier curve with control points')
hold on
plot(B(:,1),B(:,2),'MarkerFaceColor',co(counter,:)); 
% scatter(x_sol,y_sol,'filled','MarkerFaceColor',co(counter,:));
xlabel('x [m]')
ylabel('y [m]')
axis([B(1,1) B(end,1) B(1,2) B(end,2)])
fill(obs(:,1),obs(:,2),'k')

for ii=1:5:length(t)
    k = dsearchn(obs,B(ii,:));
    robot_i=B(ii,:)';
    obs_i=obs(k,:)';
    a=obs_i-robot_i;
    b=(obs_i'*obs_i-robot_i'*robot_i)/2;
    if (a'*robot_i-b<=-0.01)
        circle(B(ii,1),B(ii,2),r,co(1,:));
    else
        disp('crash')
        circle(B(ii,1),B(ii,2),r,'r');
    end
    x=0:2:2;
    y=(b-a(1)*x)/a(2);
    
    scatter(obs(k,1),obs(k,2),'*r');
%     scatter(B(ii,1),B(ii,2),'filled','MarkerFaceColor',co(counter,:));
    plot(x,y,'Color',[0.4,0.4,0.4]);
%     pause()
%     drawnow
end

hold off

subplot(2,1,2)
title('Curvature \kappa(t)')
hold on;
plot(t,kappa,'MarkerFaceColor',co(counter,:))
legend(leg(1:counter),'Position',[0.844 0.48456 0.03923 0.063096])
xlabel('t [/]')
ylabel('\kappa [1/m]')
hold off
drawnow
end


end
