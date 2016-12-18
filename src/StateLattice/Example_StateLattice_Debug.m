clear
close all
clc

%% Init
% General parameters
stateLatice=[];
tic

x1=0; y1=0; th1=-pi/2; Pstart=[x1,y1,th1]; % start sposition and orientation
xend=0; yend=1; thend=-pi/2; Pend=[xend,yend,thend]; % end position and orientation
n=4; % order of the bezier curve
t=linspace(0,1,1001)'; % overall resolution of the solver
kappa_max=1.5;
initCOP=struct('Pstart',Pstart,'Pend',Pend,'n',n,'t',t,'kappa_max',kappa_max);


optPath=BezierCOP(initCOP);
if ~isempty(optPath)
    stateLatice=[stateLatice; optPath];
    x=optPath.x;
    y=optPath.y;
    [B,dB,ddB,kappa]=BezierCurve(x,y,t);
    
    %% Plot
    co=get(gca,'ColorOrder'); % get default color for plot
    subplot(1,2,1)
    title(['Motion primitives to [', num2str(xend), ', ', num2str(yend),']'])
    hold on
    plot(B(:,1),B(:,2),'Color',co(1,:),'LineWidth',1.5); 
    grid on
    scatter(x,y,[],co(1,:),'filled')
    hold off
    xlabel('x [m]')
    ylabel('y [m]')
    axis equal

    subplot(1,2,2)
    title('Curvature \kappa(t)')
    hold on;
    grid on
    plot(t,kappa,'LineWidth',1.5)
    xlabel('t [/]')
    ylabel('\kappa [1/m]')
    hold off
end

