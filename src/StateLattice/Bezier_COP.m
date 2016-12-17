clear
close all
clc

stateLatice=[];
%% Init
% General parameters
tic
for TH1_vec= -pi/2:pi/8:pi/2 % -pi/2:pi/4:pi/2
for THend_vec=-pi/2:pi/8:pi/2
x1=0; y1=0; th1=TH1_vec; Pstart=[x1,y1,th1]; % start position and orientation
xend=2; yend=0; thend=THend_vec; Pend=[xend,yend,thend]; % end position and orientation
n=4; % order of the bezier curve
t=linspace(0,1,101)'; % overall resolution of the solver
kappa_max=1.5;
% Weigting factor for objective function
% these will be normalized (for conviniance) and rounded off and then
% checked for uniqueness
a_kappa_mat=1;
a_length_mat=0;

weights_mat=[a_kappa_mat' a_length_mat'];
weights_mat=getProcessedMat(weights_mat);
weights=weights_mat(1,:);

initCOP=struct('Pstart',Pstart,'Pend',Pend,'n',n,'t',t,'weights',weights,'kappa_max',kappa_max);

max_val=max([x1 y1 xend yend])+1;

min_val=-max_val;

%% Solver
% x
[x_init, y_init]=getInitVal(initCOP);
x=optivar(n,1,'x');

x.setInit(x_init)
x.setLb(min_val); x.setUb(max_val)
% y
y=optivar(n,1,'y');
y.setInit(y_init)
y.setLb(min_val); y.setUb(max_val)

ipoptOptions=struct('max_iter',300,'tol',1e-3,'print_level',0);
solveOptions=struct('ipopt',ipoptOptions);

sol = optisolve(objectiveFunction(x,y,initCOP),constraints(x,y,initCOP),solveOptions);
x_sol=optival(x);
y_sol=optival(y);

[satisCons] = checkConstraints(x_sol,y_sol,initCOP);
% disp(' ')
% clc
disp(' ')
if satisCons
    disp(['Found path between : ', num2str(th1*180/pi), '° and ', num2str(thend*180/pi),'°'])
    
    [B,dB,ddB,kappa]=BezierCurve(x_sol,y_sol,t);
    dx=dB(:,1);
    dy=dB(:,2);
    s=trapz(t,sqrt(dx.^2+dy.^2));
    intKappaT=trapz(t,kappa.^2);
    
    optPath=struct('th1',th1,'xend',xend,'yend',yend,'thend',thend,'x',x_sol,'y',y_sol,'s',s,'intKappaT',intKappaT);
    stateLatice=[stateLatice; optPath];
    %% Plot
    co=get(gca,'ColorOrder'); % get default color for plot

%     subplot(1,2,1)
    title('State Latice for origin to [2 2]')
    hold on
    plot(B(:,1),B(:,2),'Color',co(1,:),'LineWidth',1.5); 
    % if x1~=xend
    % else
    %     axis([x1-0.5,x1+0.5 min([x_max y_max]) max([x_max y_max])])
    % end
%     grid minor
%     scatter(x_sol,y_sol,[],co(1,:),'filled')
    hold off
    xlabel('x [m]')
    ylabel('y [m]')
%     axis([0 2 0 2])
    axis equal

%     subplot(1,2,2)
%     title('Curvature \kappa(t)')
%     hold on;
%     grid on
%     
%     plot(t,kappa,'LineWidth',1.5)
%     xlabel('t [/]')
%     ylabel('\kappa [1/m]')
%     hold off
else
    disp(['could not find path between : ', num2str(th1*180/pi), '° and ', num2str(thend*180/pi),'°'])
%     disp([th1*180/pi thend*180/pi])
end
disp(' ')
disp('============================================')
disp(' ')

% saveCurrentFigure
end
end
toc