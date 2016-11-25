clear
close all
clc

%% Init
% General parameters
x1=0; y1=0; theta1=0; Pstart=[x1,y1,theta1]; % start position and orientation
xg=2; yg=2; thetg=pi/2; Pgoal=[xg,yg,thetg]; % end position and orientation
n=4; % order of the bézier curve
t=linspace(0,1,11)'; % overall resolution of the solver
kappa_max=2;
% Weigting factor for objective function
% these will be normalized (for conviniance) and rounded off and then
% checked for uniqueness
a_kappa_mat=    [1.0 3.0 ];
a_length_mat=   [1.0 0.0 ];
a_dist_mat=     [2.0 10.0];
a_angle_mat=    [0.0 0.0 ];
weights_mat=[a_kappa_mat' a_length_mat' a_dist_mat' a_angle_mat'];
weights_mat=getProcessedMat(weights_mat);
weights=optipar(1,4);
weights.setValue(weights_mat(2,:))

initCOP=struct('Pstart',Pstart,'Pgoal',Pgoal,'n',n,'t',t,'weights',weights,'kappa_max',kappa_max);

%% Solver
x=optivar(n,1,'x');
x.setInit(getInitValX(x1,xg,n))
x.setLb(x1-1); x.setUb(xg+1)
y=optivar(n,1,'y');
y.setInit(getInitValY(y1,yg,n))
y.setLb(y1-1); y.setUb(yg+1)
sol = optisolve(objectiveFunction(x,y,initCOP),constraints(x,y,initCOP));

%% Output
x_sol=optival(x);
y_sol=optival(y);

t=linspace(0,1,1001)';
[B,~,~,kappa]=BezierCurve([x_sol y_sol],t);

disp(' ')
disp('=======================')
disp(' ')

disp(Pgoal(1:2)-B(end,:))
% disp(abs(thetag-atan(dB(end,2)/dB(end,1))))


figure(1)
subplot(2,1,1)
title('Bezier curve with control points')
hold on
plot(B(:,1),B(:,2)); 
scatter(Pstart(1),Pstart(2),'*r')
scatter(Pgoal(1),Pgoal(2),'*r')
xlabel('x [m]')
ylabel('y [m]')
axis equal
subplot(2,1,2)
title('Curvature \kappa(t)')
hold on;
plot(t,kappa)
xlabel('t [/]')
ylabel('\kappa [1/m]')
hold off