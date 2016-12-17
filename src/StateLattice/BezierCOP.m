function BezierCOP()

%% Init
% General parameters
x1=0; y1=0; th1=pi/2; Pstart=[x1,y1,th1]; % start position and orientation
xend=2; yend=2; thend=pi/4; Pend=[xend,yend,thend]; % end position and orientation
n=4; % order of the bezier curve
t=linspace(0,1,101)'; % overall resolution of the solver
kappa_max=1;
% Weigting factor for objective function
% these will be normalized (for conviniance) and rounded off and then
% checked for uniqueness
a_kappa_mat=1;
a_length_mat=0;
stateLatice=[];
weights_mat=[a_kappa_mat' a_length_mat'];
weights_mat=getProcessedMat(weights_mat);
weights=weights_mat(1,:);

initCOP=struct('Pstart',Pstart,'Pend',Pend,'n',n,'t',t,'weights',weights,'kappa_max',kappa_max);

x_min=min([x1 xend]); x_max=max([x1 xend]);
y_min=min([y1 yend]); y_max=max([y1 yend]);
%% Solver
% x
x=optivar(n,1,'x');
x.setInit(getInitValX(x1,xend,n))
x.setLb(x_min-5); x.setUb(x_max+5)
% y
y=optivar(n,1,'y');
y.setInit(getInitValY(y1,yend,n))
y.setLb(y_min); y.setUb(y_max)

ipoptOptions=struct('max_iter',500);
solveOptions=struct('ipopt',ipoptOptions);

sol = optisolve(objectiveFunction(x,y,initCOP),constraints(x,y,initCOP),solveOptions);
x_sol=optival(x);
y_sol=optival(y);

[satisCons] = checkConstraints(x_sol,y_sol,initCOP);

if satisCons
    disp(' ')
   clc
   disp('constraints satisfied')
   optPath=struct('x',x_sol,'y',y_sol,'s',1,'intKappaT',1);
   stateLatice=[stateLatice; optPath];
end

%% Plot
co=get(gca,'ColorOrder'); % get default color for plot

[B,dB,~,kappa]=BezierCurve(x_sol,y_sol,t);

subplot(1,2,1)
title('Bezier curve with control points')
hold on
plot(B(:,1),B(:,2),'LineWidth',1.5); 
% if x1~=xend
% else
%     axis([x1-0.5,x1+0.5 min([x_max y_max]) max([x_max y_max])])
% end
grid on
% plot(B(:,1),B(:,2),'Color',co(1,:),'LineWidth',1.5); 
scatter(x_sol,y_sol,[],co(1,:),'filled')
hold off
xlabel('x [m]')
ylabel('y [m]')
legend('Bezier curve','Location','NW')

subplot(1,2,2)
title('Curvature \kappa(t)')
hold on;
grid on

plot(t,kappa,'LineWidth',1.5)
xlabel('t [/]')
ylabel('\kappa [1/m]')
hold off
% saveCurrentFigure
end