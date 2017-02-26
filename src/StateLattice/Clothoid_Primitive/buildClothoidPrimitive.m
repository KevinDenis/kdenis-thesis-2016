clear
co=get(gca,'ColorOrder'); % get default color for plot
close all
clc
addpath('ClothoidG1fitting');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                      %
%            State Latice Matrix Structure             %
%  [ x0 y0 th0 x1 y1 th1 kappa dkappa Ltot intKappa ]  %
%    1  2   3  4  5   6    7     8    9    10          %
%                                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Init
% General parameters
x_max=1;
dx=0.1;
dx_show=0.2;
X=0:dx:x_max;
Y=X;
dth=pi/16;
dth_show=pi/8;
TH0=0:dth:pi/2;
TH1=0:dth:pi/2;
grid_TH0_X_Y_TH1=getAllComb(TH0,X,Y,TH1);
grid_X_Y = getAllComb(X,Y);
grid_X_Y_L1 = abs(grid_TH0_X_Y_TH1(:,2))+abs(abs(grid_TH0_X_Y_TH1(:,3)));
x0=0; 
y0=0; 
NPTS=100;
kappa_max=1;
ClothoidStateLaticeQ1=zeros(1,10);
n=1;
for ii=1:2*x_max/dx
    kk=IsNear(grid_X_Y_L1,ii*dx,1e-3);
    grid_kk=grid_TH0_X_Y_TH1(kk,:);
    for jj=1:size(grid_kk,1)
        th0=grid_kk(jj,1);
        x1=grid_kk(jj,2); 
        y1=grid_kk(jj,3); 
        th1=grid_kk(jj,4);
        [kappa,dkappa,Ltot,iter] = buildClothoid(x0,y0,th0,x1,y1,th1);
        if abs(kappa) <= kappa_max && abs(kappa+dkappa) <= kappa_max
            s=linspace(0,Ltot,NPTS);
            ks=kappa+dkappa*s;
            intKappa=trapz(s,abs(ks));
            ClothoidStateLaticeQ1(n,:)=[x0 y0 th0 x1 y1 th1 kappa dkappa Ltot intKappa];
            n=n+1;
        end
    end
end


ClothoidStateLaticeQ2=ClothoidStateLaticeQ1;
ClothoidStateLaticeQ2(:,3)=pi-ClothoidStateLaticeQ1(:,3);
ClothoidStateLaticeQ2(:,4)=-ClothoidStateLaticeQ1(:,4);
ClothoidStateLaticeQ2(:,5)=ClothoidStateLaticeQ1(:,5);
ClothoidStateLaticeQ2(:,6)=pi-ClothoidStateLaticeQ1(:,6);
ClothoidStateLaticeQ2(:,7)=-ClothoidStateLaticeQ1(:,7);
ClothoidStateLaticeQ2(:,8)=-ClothoidStateLaticeQ1(:,8);

ClothoidStateLaticeQ3=ClothoidStateLaticeQ1;
ClothoidStateLaticeQ3(:,3)=-pi+ClothoidStateLaticeQ1(:,3);
ClothoidStateLaticeQ3(:,4)=-ClothoidStateLaticeQ1(:,4);
ClothoidStateLaticeQ3(:,5)=-ClothoidStateLaticeQ1(:,5);
ClothoidStateLaticeQ3(:,6)=-pi+ClothoidStateLaticeQ1(:,6);

ClothoidStateLaticeQ4=ClothoidStateLaticeQ1;
ClothoidStateLaticeQ4(:,3)=-ClothoidStateLaticeQ1(:,3);
ClothoidStateLaticeQ4(:,5)=-ClothoidStateLaticeQ1(:,5);
ClothoidStateLaticeQ4(:,6)=-ClothoidStateLaticeQ1(:,6);
ClothoidStateLaticeQ4(:,7)=-ClothoidStateLaticeQ1(:,7);
ClothoidStateLaticeQ4(:,8)=-ClothoidStateLaticeQ1(:,8);


ClothoidStateLatice_rot0=[ClothoidStateLaticeQ1;ClothoidStateLaticeQ2;ClothoidStateLaticeQ3;ClothoidStateLaticeQ4];

ClothoidStateLatice_rot90=ClothoidStateLatice_rot0;
ClothoidStateLatice_rot90(:,3)=pi/2+ClothoidStateLatice_rot0(:,3);
ClothoidStateLatice_rot90(:,4)=-ClothoidStateLatice_rot0(:,5); % (-y, x)
ClothoidStateLatice_rot90(:,5)=ClothoidStateLatice_rot0(:,4); % (-y, x)
ClothoidStateLatice_rot90(:,6)=pi/2+ClothoidStateLatice_rot0(:,6);

ClothoidStateLatice_rot180=ClothoidStateLatice_rot90;
ClothoidStateLatice_rot180(:,3)=pi/2+ClothoidStateLatice_rot90(:,3);
ClothoidStateLatice_rot180(:,4)=-ClothoidStateLatice_rot90(:,5);
ClothoidStateLatice_rot180(:,5)=ClothoidStateLatice_rot90(:,4);
ClothoidStateLatice_rot180(:,6)=pi/2+ClothoidStateLatice_rot90(:,6);


ClothoidStateLatice_rot270=ClothoidStateLatice_rot180;
ClothoidStateLatice_rot270(:,3)=pi/2+ClothoidStateLatice_rot180(:,3);
ClothoidStateLatice_rot270(:,4)=-ClothoidStateLatice_rot180(:,5);
ClothoidStateLatice_rot270(:,5)=ClothoidStateLatice_rot180(:,4);
ClothoidStateLatice_rot270(:,6)=pi/2+ClothoidStateLatice_rot180(:,6);


ClothoidStateLaticeFull=[ClothoidStateLatice_rot0;ClothoidStateLatice_rot90;ClothoidStateLatice_rot180;ClothoidStateLatice_rot270];
ClothoidStateLaticeFull(:,3)=wrapToPi(ClothoidStateLaticeFull(:,3));
ClothoidStateLaticeFull(:,6)=wrapToPi(ClothoidStateLaticeFull(:,6));
ClothoidStateLaticeFull(ClothoidStateLaticeFull(:,3)==-pi,3)=pi;
ClothoidStateLaticeFull(ClothoidStateLaticeFull(:,6)==-pi,6)=pi;


length(ClothoidStateLaticeFull)
[~,idx,~] =  unique(round(ClothoidStateLaticeFull(:,1:6),3),'rows'); ClothoidStateLaticeFull=ClothoidStateLaticeFull(idx,:);
length(ClothoidStateLaticeFull)

if true
% Plot
% scatter(grid_X_Y(:,1), grid_X_Y(:,2)) ;          
grid on
xlabel('x [m]')
ylabel('y [m]')
axis equal
hold on
ClothSL_tmp=ClothoidStateLaticeFull;

% progressbar('Drawing Clothoid State Lattice') 
for ii=1:1:length(ClothSL_tmp)
    x0=ClothSL_tmp(ii,1);
    y0=ClothSL_tmp(ii,2);
    th0=ClothSL_tmp(ii,3);
    x1=ClothSL_tmp(ii,4);
    y1=ClothSL_tmp(ii,5);
    th1=ClothSL_tmp(ii,6);
    %IsNear(rem(th0*180/pi,45),0,1e-2)
    %IsNear(th0*180/pi,90,1e-2)
    if IsNear(rem(th0,dth_show),0,1e-2) && IsNear(rem(x0,dx_show),0,1e-2)  && IsNear(rem(y0,dx_show),0,1e-2) && ... 
       IsNear(rem(th1,dth_show),0,1e-2) && IsNear(rem(x1,dx_show),0,1e-2)  && IsNear(rem(y1,dx_show),0,1e-2)
        k=ClothSL_tmp(ii,7);
        dk=ClothSL_tmp(ii,8);
        Lsol=ClothSL_tmp(ii,9);
        [X,Y] = pointsOnClothoid(x0,y0,th0,k,dk,Lsol,NPTS) ;
        figure(1)
        hold on
        if ~IsNear(dk,0,1e-3)
            plot(X,Y,'Color',co(1,:),'Linewidth',1);
        else
            plot(X,Y,'Color',co(2,:),'Linewidth',2) ;
        end
        grid on
        xlabel('x [m]')
        ylabel('y [m]')
        axis equal
        hold off
    end
%     progressbar(ii/length(ClothSL_tmp))
end
l1=legend('constant \kappa(s)','Linearly varying \kappa(s)');
set(l1,'FontSize',12);
saveCurrentFigure

end

