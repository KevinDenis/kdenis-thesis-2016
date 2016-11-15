clear
close all
clc

%% Init
t = linspace(0,1,1001)';
P1=[0 0];
P2=[1 1];
P3=[1 2];
P4=[2 2];
P=[P1;P2;P3;P4];

%% B�zier
[B,dB,ddB,kappa] = BezierCurve(P,t);
dB1yx= dB(:,2)./dB(:,1);
ddB1yx= ddB(:,2)./ddB(:,1);
xx=B(:,1);
yy=B(:,2);
dxx=dB(:,1);
dyy=dB(:,2);

%% Finite Diferences
dB2y=fdm(yy,[1:length(yy)]',1,5);
dB2x=fdm(xx,[1:length(xx)]',1,5);
dB2yx=dB2y./dB2x;

ddB2y=fdm(yy,[1:length(yy)]',2,5);
ddB2x=fdm(xx,[1:length(xx)]',2,5);
ddB2yx=ddB2y./ddB2x;
kappa2=((dB2x.*ddB2y-dB2y.*ddB2x))./((dB2x.^2+dB2y.^2).^(3/2));

%% Control
assert(max(dB1yx- dB2yx)<1e-5)
assert(max(ddB1yx- ddB2yx)<1e-5)
assert(max(kappa- kappa2)<1e-5)

%% Distance calc
s =trapz(t,sqrt(dxx.^2+dyy.^2));
s_control=trapz(t(1:end-1),sqrt((diff(xx)./diff(t)).^2+(diff(yy)./diff(t)).^2));
disp(max((s-s_control)/s_control))

%% Kappa
s=cumtrapz(t,sqrt(dxx.^2+dyy.^2));
intKappaT=cumtrapz(t,kappa.^2)
intKappaS=cumtrapz(s,kappa.^2)
intKappaT_S=intKappaT*s(end);



%% Plot stuff
% dB_norm =
dB_norm=max(sqrt(dB(:,1).^2+dB(:,2).^2));
dB_vec=[dB(:,1) dB(:,2)]/dB_norm;
ddB_norm=max(sqrt(ddB(:,1).^2+ddB(:,2).^2));
ddB_vec=[ddB(:,1) ddB(:,2)]/ddB_norm;

figure(1)
hold on
% for ii=1:2:length(xx)
%     quiver(xx(ii),yy(ii),dB_vec(ii,1),dB_vec(ii,2),'g')
%     quiver(xx(ii),yy(ii),ddB_vec(ii,1),ddB_vec(ii,2),'r')
% end
plot(xx,yy,'b','Linewidth',3); 
scatter(P(:,1),P(:,2),'k','filled')
axis equal

figure(2)
plot(s,intKappaT_S); hold on
plot(s,intKappaS);
