clear
close all
clc

%% Init
% General parameters
x1=0; y1=0; theta1=0; Pstart=[x1,y1,theta1]; % start position and orientation
xg=2; yg=2; thetg=pi/2; Pgoal=[xg,yg,thetg]; % end position and orientation
n=4; % order of the bezier curve
t=linspace(0,1,201)'; % overall resolution of the solver
kappa_max=3;
% Weigting factor for objective function
% these will be normalized (for conviniance) and rounded off and then
% checked for uniqueness
a_kappa_mat=    [ 2.0  1.0];
a_length_mat=   [ 0.5  1.0];
a_dist_mat=     [ 5.0  5.0];
a_angle_mat=    [ 0.0  0.5];
weights_mat=[a_kappa_mat' a_length_mat' a_dist_mat' a_angle_mat'];
weights_mat=getProcessedMat(weights_mat);
% weights=optipar(1,4);
% weights.setValue(weights_mat(1,:))
weights=weights_mat(1,:);

[bezierMatrix,zeroToZMatrix,zToTMatrix]=getBezierMatrix(n-1,0.5);

obs=[1.0 0.5;1.5 0.5;1.5 0.0;1.0 0.0]-0.2;
% obs_filled=[];
% for ii=1:length(obs)-1
%     obs_filled=[obs_filled;fillline(obs(ii,:),obs(ii+1,:),40)];
% end
% obs_filled=[obs_filled;fillline(obs(end,:),obs(1,:),40)];
% obs=obs_filled;

initCOP=struct('Pstart',Pstart,'Pgoal',Pgoal,'n',n,'t',t,'weights',weights,'kappa_max',kappa_max,'obs',obs,'zeroToZMatrix',zeroToZMatrix,'zToTMatrix',zToTMatrix);

%% Solver
% x
x=optivar(n,1,'x');
x.setInit(getInitValX(x1,xg,n))
% x.setInit([0;0.4287;1.2994;1.3760]);
x.setLb(x1-1); x.setUb(xg+1)
% y
y=optivar(n,1,'y');
y.setInit(getInitValY(y1,yg,n))
% y.setInit([0;0;1;1]);
y.setLb(y1-1); y.setUb(yg+1)
% a
a=optivar(2,2,'a');
a.setInit([1 1;0 0])
a.setLb(-1); a.setUb(1)
% b
b=optivar(2,1,'b');
b.setInit([0 0])
% b.setLb(y1-1); a.setUb(yg+1)

sol = optisolve(objectiveFunction(x,y,initCOP),constraints(x,y,a,b,initCOP));
x_sol=optival(x);
y_sol=optival(y);
a_sol=optival(a);
b_sol=optival(b);

sepLineX1=0:0.1:2;
sepLineY1=(b_sol(1)-a_sol(1,1)*sepLineX1)/a_sol(2,1);


sepLineX2=0:0.1:2;
sepLineY2=(b_sol(2)-a_sol(1,2)*sepLineX2)/a_sol(2,2);

x_start=zeroToZMatrix*x_sol;
x_end=zToTMatrix*x_sol;
y_start=zeroToZMatrix*y_sol;
y_end=zToTMatrix*y_sol;




%% Plot
co=get(gca,'ColorOrder'); % get default color for plot

[B,~,~,kappa]=BezierCurve(x_sol,y_sol,t);
[B_start,~,~,~]=BezierCurve(x_start,y_start,t);
[B_end,~,~,~]=BezierCurve(x_end,y_end,t);


% disp(' '); disp('======================='); disp(' ')
% disp(Pgoal(1:2)-B(end,:))
% disp(abs(thetag-atan(dB(end,2)/dB(end,1))))

figure(1)
% subplot(2,1,1)
title('Bezier curve with control points')
hold on
fill(x_sol,y_sol,co(1,:),'FaceAlpha',0.2)

fill(x_start,y_start,co(3,:),'FaceAlpha',0.2)
plot(B_start(:,1),B_start(:,2),'Color',co(3,:),'LineWidth',1.5); 
scatter(x_start,y_start,[],co(3,:),'filled')

fill(x_end,y_end,co(4,:),'FaceAlpha',0.2)
plot(B_end(:,1),B_end(:,2),'Color',co(4,:),'LineWidth',1.5); 
scatter(x_end,y_end,[],co(4,:),'filled')

scatter([Pstart(1) Pgoal(1)],[Pstart(2) Pgoal(2)],100,co(2,:),'Marker','*')
fill(obs(:,1),obs(:,2),'k')

plot(sepLineX1,sepLineY1,'Color',co(3,:),'LineWidth',1.5)
plot(sepLineX2,sepLineY2,'Color',co(4,:),'LineWidth',1.5)

xlabel('x [m]')
ylabel('y [m]')
axis([-0.5 2.5 -0.5 2.5])
axis equal
legend('Convex hull', ...
        'Convex hull 1','Splitted Bezier curve 1','Splitted Bezier Control Point 1',...
        'Convex hull 2','Splitted Bezier curve 2','Splitted Bezier Control Point 2',...
        'Start-Goal position ','Obstacle','Separating Hyperplane 1','Separating Hyperplane 2', 'Location','NW')
% subplot(2,1,2)
% title('Curvature \kappa(t)')
% hold on;
% plot(t,kappa)
% xlabel('t [/]')
% ylabel('\kappa [1/m]')
% hold off