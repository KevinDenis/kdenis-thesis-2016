clear
co=get(gca,'ColorOrder');
close all
clc

%% Init
% General parameters
stateLatice=[];
x_max=2;
dx=0.5;
X_vec=0:dx:x_max;
Y_vec=X_vec;
dth=pi/4;
TH1_vec=-pi/2:dth:pi/2;
THend_vec=TH1_vec;
grid_TH1_X_Y_THend=getAllComb(TH1_vec,X_vec,Y_vec,THend_vec);
grid_X_Y_L1 = abs(grid_TH1_X_Y_THend(:,2))+abs(abs(grid_TH1_X_Y_THend(:,3)));

progressbar('Overall State Lattice Progress','Current Manhattan Distance Progress')

for ii=1/dx:2*x_max/dx
    kk=IsNear(grid_X_Y_L1,ii*dx,1e-3);
    grid_kk=grid_TH1_X_Y_THend(kk,:);
%     colorIdx=1+rem((ii-1/dx),7);
    figure(2)
    hold on
    title(['Motion primitives origin to Manhattan Distance ', num2str(ii*dx),' (\Delta\theta = ',  num2str(round(dth*180/pi)),'°)'])
    for jj=1:size(grid_kk,1)
        x1=0; y1=0; th1=grid_kk(jj,1); Pstart=[x1,y1,th1]; % start sposition and orientation
        xend=grid_kk(jj,2); yend=grid_kk(jj,3); thend=grid_kk(jj,4); Pend=[xend,yend,thend]; % end position and orientation
        n=4; % order of the bezier curve
        t=linspace(0,1,101)'; % overall resolution of the solver
        kappa_max=1.5;
        initCOP=struct('Pstart',Pstart,'Pend',Pend,'n',n,'t',t,'kappa_max',kappa_max);

        optPath=BezierCOP(initCOP);
        frac2 = jj/size(grid_kk,1);
        frac1 = (ii-1/dx+frac2)/(2*x_max/dx-1/dx+1);
        progressbar(frac1, frac2)
        if ~isempty(optPath)
            stateLatice=[stateLatice; optPath];
            x=optPath.x;
            y=optPath.y;
            [B,dB,ddB,kappa]=BezierCurve(x,y,t);
            %% Plot
            plot(B(:,1),B(:,2),'Color',co(1,:),'LineWidth',1.5); 
            grid on
            xlabel('x [m]')
            ylabel('y [m]')
            axis equal
        end
    end
    hold off

    saveCurrentFigure
    close gcf

end