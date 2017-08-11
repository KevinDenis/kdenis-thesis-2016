initPlotScripts
res=0.01;
n=500; % there are 500 paths in total, forward and backward.
th=linspace(0,2*pi,n+1); th(end)=[];
v_all=sin(th);
w_all=cos(th);
k=w_all./v_all;

idxKeep=abs(k)<=1;
k=k(idxKeep);
v_valid=v_all(idxKeep);
w_valid=w_all(idxKeep);
v_disc=v_all(~idxKeep);
w_disc=w_all(~idxKeep);

robotPose=[0 0 0];

LSL(1:length(v_valid),1)=struct('x0',0,'y0',0,'th0',0,...
                         'x1',[],'y1',[],'th1',[],...
                         'X',[],'Y',[],'TH',[],...
                         'S',[],'K',[], ... 
                         'k',0,'dk',0,'Ltot',0,'intK',0,...
                         'PathOccXY',[], 'pathCost',[], ...
                          'free',true,'idxBlocked',[],'ID',[]);
for ii=1:length(v_valid)
    v_ii=v_valid(ii);
    w_ii=w_valid(ii);
    dt=abs(res/v_ii);
    t=0:dt:4;
    m=length(t);
    X=zeros(m,1);
    Y=zeros(m,1);
    TH=zeros(m,1);
    S=zeros(m,1);
    K=repelem(k(ii),m,1);
    for jj=2:length(t)
        dx=cos(TH(jj-1))*v_ii*dt;
        dy=sin(TH(jj-1))*v_ii*dt;
        dth=w_ii*dt;
        X(jj)=X(jj-1)+dx;
        Y(jj)=Y(jj-1)+dy;
        TH(jj)= TH(jj-1)+dth;
        S(jj)=S(jj-1)+sqrt(dx^2+dy^2);
    end
    Ltot=S(end);
    LSL(ii).x0 = 0;
    LSL(ii).y0 = 0;
    LSL(ii).th0= 0;
    LSL(ii).x1 = X(end);
    LSL(ii).y1 = Y(end);
    LSL(ii).th1= TH(end);
    LSL(ii).X  = X;
    LSL(ii).Y  = Y;           
    LSL(ii).TH = TH;
    LSL(ii).S = S;
    LSL(ii).K = K;
    LSL(ii).k  = k(ii);
    LSL(ii).dk = 0;
    LSL(ii).Ltot=Ltot;
    LSL(ii).free=true;
end

fig=figureFullScreen();
subplot(1,2,1)
title('')
hold on
xlabel('\omega [rad/sec]')
ylabel('v [m/s]')
plot([w_valid w_valid(1)],[v_valid v_valid(1)],'.','Color',co(1,:),'MarkerSize',20)
% plot([w_disc w_disc(1)],[v_disc v_disc(1)],'.','Color',co(2,:),'MarkerSize',20)
axis equal
% l=legend('Valid input velocities','Discarded input velocities','Location','E');
l=legend('Input velocities','Location','E');
set(l,'FontSize',26);
set(gca,'FontSize',24)
axis(1.1*[-1 1 -1 1])

subplot(1,2,2)
title('')
hold on
xlabel('x [m]')
ylabel('y [m]')
plotPath(LSL,co(1,:),2)
plotRoboticWheelchair(robotPose)
axis equal
l=legend('Circular Paths','Robot pose','Location','SE');
set(l,'FontSize',26);
set(gca,'FontSize',24)
axis([-4 4 -4 4])

saveCurrentFigure('MPCircVWPair_Eval');