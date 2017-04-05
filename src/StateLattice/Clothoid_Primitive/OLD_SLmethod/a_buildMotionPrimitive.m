% Notes :
%   *
%   *

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                               %
%                    State Latice Structure                     %
%   [ x0 y0 th0 x1 y1 th1 X Y TH kappa dkappa Ltot intKappa ]   %
%                                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clearvars
co=get(gca,'ColorOrder'); % get default color for plot
close all
clc
addpath('ClothoidG1fitting');

%% Init
% General parameters

npts=100;
res=5e-2;

showPlot=true;
x_max=2;
dx=0.25;
dx_show=0.5;
X=0:dx:x_max;
Y=X;
dth=pi/8;
dth_show=2*dth;
TH0=0:dth:pi/2;
TH1=TH0;
grid_TH0_X_Y_TH1=getAllComb(TH0,X,Y,TH1);
grid_X_Y = getAllComb(X,Y);
grid_L1 = abs(grid_TH0_X_Y_TH1(:,2))+abs(abs(grid_TH0_X_Y_TH1(:,3)));
x0=0; 
y0=0; 
kappa_max=1;


MotionPrem(1:2,1)=struct('x0',0,'y0',0,'th0',0,...
                         'x1',0,'y1',0,'th1',0,...
                         'X',zeros(npts,1),'Y',zeros(npts,1),'TH',zeros(npts,1),...
                         'k',0,'dk',0,'Ltot',0,'intK',0,...
                         'PathOccXY',[], 'pathCost',[],'free',1,'ID',1);
nn=1;
for ii=1:2*x_max/dx
    kk=IsNear(grid_L1,ii*dx,1e-3);
    grid_kk=grid_TH0_X_Y_TH1(kk,:);
    for jj=1:size(grid_kk,1)
        th0=grid_kk(jj,1);
        x1=grid_kk(jj,2);
        y1=grid_kk(jj,3);
        th1=grid_kk(jj,4);
        [k,dk,Ltot,~] = buildClothoid(x0,y0,th0,x1,y1,th1);
        if abs(k) <= kappa_max && abs(k+dk*Ltot) <= kappa_max
            npts=round(Ltot/res);
            s=linspace(0,Ltot,npts);
            ks=k+dk*s;
            intK=trapz(s,abs(ks));
            [X,Y]=pointsOnClothoid(x0,y0,th0,k,dk,Ltot,npts);
            TH=wrapToPi(cumtrapz(s,ks)+th0);
            TH(1)=th0; 
            TH(end) = th1;
            MotionPrem(nn).x0 = x0;
            MotionPrem(nn).y0 = y0;
            MotionPrem(nn).th0= th0;
            MotionPrem(nn).x1 = x1;
            MotionPrem(nn).y1 = y1;
            MotionPrem(nn).th1= th1;
            MotionPrem(nn).X  = X.';
            MotionPrem(nn).Y  = Y.';           
            MotionPrem(nn).TH = TH.';
            MotionPrem(nn).k  = k;
            MotionPrem(nn).dk = dk;
            MotionPrem(nn).Ltot=Ltot;
            MotionPrem(nn).intK=intK;
            MotionPrem(nn).pathCost=3*Ltot+intK;
            MotionPrem(nn).free=true;
            MotionPrem(nn).ID=nn;
            nn=nn+1;
        end
    end
end

[MotionPrem] = MirrorRotateMotionPrem(MotionPrem);
% CheckStructure(MotionPrem)

%% Plot Motion Primitive
if showPlot
    figure()
    hold on
    grid on
    xlabel('x [m]')
    ylabel('y [m]')
    axis equal

    [MotionPrem_k_cte,MotionPrem_k_notcte] = sortMotionPremPlot(MotionPrem,dx_show,dth_show);
    
    %% Plot Clothiodes
    for ii=1:1:length(MotionPrem_k_notcte)
%         if IsNear(MotionPrem_k_notcte(ii).th0*180/pi,90,1e-2)
            plot(MotionPrem_k_notcte(ii).X,MotionPrem_k_notcte(ii).Y,'Color',co(1,:),'Linewidth',1);
%         end
    end
    
    %% Plot Straight Lines and Circles
    for ii=1:1:length(MotionPrem_k_cte)
%         if IsNear(MotionPrem_k_cte(ii).th0*180/pi,90,1e-2)
            plot(MotionPrem_k_cte(ii).X,MotionPrem_k_cte(ii).Y,'Color',co(2,:),'Linewidth',2);
%         end 
    end

%     l1=legend('constant \kappa(s)','Linearly varying \kappa(s)');
%     set(l1,'FontSize',12);
%     saveCurrentFigure

end
%}