clear
close all
clc
import casadi.*

co=get(gca,'ColorOrder');

t=linspace(0,1,11);
P = MX.sym('P',4,2);
P_val=[0,0;1,0;2,1;2,2];
P(:,:)=P_val;
[B,dB,ddB,kappa]=BezierCurve(P,t);
obs = MX.sym('obs',4,2);
obs_val=   [1.0 0.5;
            1.5 0.5;
            1.5 0.0;
            1.0 0.0];
obs(:,:)=obs_val;
% obs_filled=[];
% for ii=1:length(obs)-1
%     obs_filled=[obs_filled;fillline(obs(ii,:),obs(ii+1,:),40)];
% end
% obs_filled=[obs_filled;fillline(obs(end,:),obs(1,:),40)];
% obs=obs_filled;

%% obstacle hyperplane

[k,d] = dsearchnCasadi(obs,P);
C=zeros(length(k),1);
C_temp=zeros(1,size(obs,1));
robot_k=B;
obs_k=obs(k,:);
% a=obs_k-robot_k;
for ii=1:length(k)
    C_temp(1)=(obs_k(ii,:)-robot_k(ii,:))*(obs_k(1,:)-(obs_k(ii,:)+robot_k(ii,:))/2)'-.01;
    C_temp(2)=(obs_k(ii,:)-robot_k(ii,:))*(obs_k(2,:)-(obs_k(ii,:)+robot_k(ii,:))/2)'-.01;
    C_temp(3)=(obs_k(ii,:)-robot_k(ii,:))*(obs_k(3,:)-(obs_k(ii,:)+robot_k(ii,:))/2)'-.01;
    C_temp(4)=(obs_k(ii,:)-robot_k(ii,:))*(obs_k(4,:)-(obs_k(ii,:)+robot_k(ii,:))/2)'-.01;
    C(ii)=min(C_temp);    
end

crash=find(C<0);

figure(1)
title('Bezier curve with control points')
hold on
fill(obs(:,1),obs(:,2),'k');
plot(B(:,1),B(:,2),'Color',co(1,:));
scatter(B(crash,1),B(crash,2),[],co(2,:),'filled');
scatter(P(:,1),P(:,2),[],co(1,:),'filled');
xlabel('x [m]')
ylabel('y [m]')
axis equal