clear
close all
clc

stateLatice=[];
%% Init
% General parameters
tic

% toPoint=[1,0;1,1;2,0;2,2];

toPoint=[2,0];
dth=pi/8;

for ii =1:size(toPoint,1)
xend_ii=toPoint(ii,1);
yend_ii=toPoint(ii,2);
for TH1_vec=-pi/2:dth:pi/2 % -pi/2:pi/4:pi/2
for THend_vec=-pi/2:dth:pi/2
x1=0; y1=0; th1=TH1_vec; Pstart=[x1,y1,th1]; % start sposition and orientation
xend=xend_ii; yend=yend_ii; thend=THend_vec; Pend=[xend,yend,thend]; % end position and orientation
n=4; % order of the bezier curve
t=linspace(0,1,101)'; % overall resolution of the solver
kappa_max=1.5;
initCOP=struct('Pstart',Pstart,'Pend',Pend,'n',n,'t',t,'kappa_max',kappa_max);


optPath=BezierCOP(initCOP);
if ~isempty(optPath)
    stateLatice=[stateLatice; optPath];
    x=optPath.x;
    y=optPath.y;
    [B,dB,ddB,kappa]=BezierCurve(x,y);
    
    %% Plot
    co=get(gca,'ColorOrder'); % get default color for plot
%     subplot(1,2,1)
    title(['Motion primitives origin to [', num2str(xend), ', ', num2str(yend),'] with \Delta\theta = ',  num2str(round(dth*180/pi)),'°'])
    hold on
    plot(B(:,1),B(:,2),'Color',co(1,:),'LineWidth',1.5); 
    grid on
%     scatter(x,y,[],co(1,:),'filled')
    hold off
    xlabel('x [m]')
    ylabel('y [m]')
    axis equal

%     subplot(1,2,2)
%     title('Curvature \kappa(t)')
%     hold on;
%     grid on
%     plot(t,kappa,'LineWidth',1.5)
%     xlabel('t [/]')
%     ylabel('\kappa [1/m]')
%     hold off
%     pause()
end

end
end
toc
saveCurrentFigure
close all

end