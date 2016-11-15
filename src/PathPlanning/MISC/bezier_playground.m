close all
clear
clc


t=0:0.001:1;

P1=[0 0];
P2=[0 1];
P3=[1 2];
P4=[2 2];
P=[P1;P2;P3;P4];


Q1=P4;
Q2=2*P4-P3;
Q3=P2+4*(P4-P3);
Q4=[4.5 0];
Q=[Q1;Q2;Q3;Q4];

x1=10*P(:,1);
y1=10*P(:,2);


[B,dB,ddB,kappa]=BezierCurve(P,t);

xx=ddB(:,1);
yy=ddB(:,2);

figure(1)
hold on
plot(xx,yy)
scatter(P(:,1),P(:,2))

% dB_norm =
dB_norm=max(sqrt(norm(dB(:,1)).^2+norm(dB(:,2)).^2));
dB_vec=[dB(:,1) dB(:,2)]/dB_norm*5;
ddB_norm=max(sqrt(norm(ddB(:,1)).^2+norm(ddB(:,2)).^2));
ddB_vec=[ddB(:,1) ddB(:,2)]/ddB_norm*5;

figure(2)
hold on
plot(xx,yy); 
scatter(P(:,1),P(:,2))
for ii=1:5:length(xx)
%     plot([xx(ii) cx(ii)], [yy(ii) cy(ii)],'g--o')
%     quiver(xx(ii),yy(ii),dB_vec(ii,1),dB_vec(ii,2),'g')
%     quiver(xx(ii),yy(ii),ddB_vec(ii,1),ddB_vec(ii,2),'r')
end


