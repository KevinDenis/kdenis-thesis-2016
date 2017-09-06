initWorkspace
co=get(groot,'DefaultAxesColorOrder');
if ~exist('LSL', 'var'); load('LSL_cloth_backup.mat'); end
if ~exist('ObstacleTable', 'var'); load('ObstacleTable_cloth.mat');  end
if ~exist('XY_ObsTable', 'var'); load('XY_ObsTable_cloth.mat'); end

load('OVP_sol.mat')

robotPose = [0 0 0];
RobotHull=[-0.80  0.20;
           -0.46  0.30;
            0.50  0.30;
            0.50 -0.30;
           -0.46 -0.30;
           -0.80 -0.20;
           -0.80  0.20];

MP=getMotionPrimitiveFromStateLattice(LSL);
MP_fwd=getForwardMotionFromStateLattice(MP);
Path=MP_fwd(136);

verticesMatrix = [[MP_fwd.x1].' [MP_fwd.y1].'];
idx=find(ismember(verticesMatrix,[2 1.5],'rows'));
% findrow_mex(verticesMatrix,[2.4000 2.5500 pi])


% XY_occ_obs=getXYobsFromBMP('obs_star.bmp');


%% Path Info and time discritisation
PathOccGrid_XY_idx=Path.PathOccXY;
PathOccGrid_XY = PathOccGrid_XY_idx(:,1:2);
dt = 2e-2/2;
% dt = 0.1;
t=0:dt:20;

%% Dynamic Obstacle Info
%Circle
XY_occ_obs_circ=getXYobsFromBMP('obs_circle.bmp');
C_xy_start_obs_circ = [0.5 1.5];
V_c_obs_circ = 1.5*[0 -0.25]; 
[firstImpactTime_circ,lastImpactTime_circ]=getTimeFirstLastImpact(PathOccGrid_XY,XY_occ_obs_circ,C_xy_start_obs_circ,V_c_obs_circ,t);
t_impact_circ = firstImpactTime_circ:dt:lastImpactTime_circ;
[TS_occ_circ,ConvHull_kk_circ]=getTSOccGrid(t_impact_circ,Path,RobotHull,XY_occ_obs_circ,C_xy_start_obs_circ,V_c_obs_circ);

%Rectangle
XY_occ_obs_rec=getXYobsFromBMP('obs_rectangle.bmp');
C_xy_start_obs_rec = [3.5 0.75];
V_c_obs_rec = [-1 0]; 
[firstImpactTime_rec,lastImpactTime_rec]=getTimeFirstLastImpact(PathOccGrid_XY,XY_occ_obs_rec,C_xy_start_obs_rec,V_c_obs_rec,t);
t_impact_rec = firstImpactTime_rec:dt:lastImpactTime_rec;
[TS_occ_rec,ConvHull_kk_rec]=getTSOccGrid(t_impact_rec,Path,RobotHull,XY_occ_obs_rec,C_xy_start_obs_rec,V_c_obs_rec);


%% Dynamics
N = 50;         %discretization points
c = 5;         %damping coefficient [Ns/m]
m = 200;        % wheelchair mass [kg]
d = Path.S(end);     %distance to travel[m]
Fmin = -250; %minimum force[N]
Fmax =  250; %maximum force [N]

TS_occ_rec_convex_simple=simplifyConvexHull(TS_occ_rec(ConvHull_kk_rec,:));
TS_occ_circ_convex_simple=simplifyConvexHull(TS_occ_circ(ConvHull_kk_circ,:));

obs_rec = TS_occ_rec_convex_simple;
obs_circ = TS_occ_circ_convex_simple;
% Ode description of car model
ode = @(x,u) [x(2);(u-c*x(2))/m];

% Decision variables
s = optivar(2,N+1); %states [s, s_dot]
a_rec = optivar(N,2);
b_rec = optivar(N,1);
a_circ = optivar(N,2);
b_circ = optivar(N,1);
F = optivar(N,1);   %input
T = optivar();    %motion time

h = T/N; %discretization step

%build constraints
con = {}; 
for k=1:N
    xp = s(:,k) + h*ode(s(:,k),F(k)); %Euler integration
%     xp = rk_step(ode,s(:,k),F(k),h); %Runge-Kutta integration
    con = {con{:}, xp == s(:,k+1)};   %impose system dynamics
    
    con = {con{:}, a_rec(k,:)*[h*k s(1,k)].' - b_rec(k,:) <= -0.125};
    for ii=1:(length(obs_rec)-1)
        con = {con{:}, a_rec(k,:)*obs_rec(ii,:).' - b_rec(k,:) >= 0};
    end
    con = {con{:}, a_rec(k,:)*a_rec(k,:).' <= 1};
    
    con = {con{:}, a_circ(k,:)*[h*k s(1,k)].' - b_circ(k,:) <= -0.125};
    for ii=1:(length(obs_circ)-1)
        con = {con{:}, a_circ(k,:)*obs_circ(ii,:).' - b_circ(k,:) >= 0};
    end
    con = {con{:}, a_circ(k,:)*a_circ(k,:).' <= 1};
end

con = {con{:}, F<= Fmax, F>=Fmin};%force constraints
con = {con{:}, T>=0}; %positive motion time
con = {con{:},	s(:,1)==[0;1.00],	s(1,end)==d,	s(2,:)<=1.0,	s(2,:)>=0}; %initial and final conditions

%necessary?
T.setInit(4.2); %assign initial guess to T
% s.setInit([linspace(0,d,N+1);ones(1,N+1)]); %assign initial guess to states

s.setInit(s_sol); %assign initial guess to states
F.setInit(F_sol); %initial guess for force
a_rec.setInit(a_sol_rec); %initial guess
b_rec.setInit(b_sol_rec); %initial guess
a_circ.setInit(a_sol_circ); %initial guess
b_circ.setInit(b_sol_circ); %initial guess

optisolve(T+(1-s(2,:))*(1-s(2,:))', con); %solve optimization problem
t_sol = linspace(0,optival(T),N+1);
s_sol = optival(s); %get value of casADi variable
F_sol = optival(F);
a_sol_rec = optival(a_rec);
b_sol_rec = optival(b_rec);
a_sol_circ = optival(a_circ);
b_sol_circ = optival(b_circ);
 
disp(strcat('Optimal motion time: ' , num2str(optival(T)), ' s'));

save('matlab.mat','C_xy_start_obs_circ','C_xy_start_obs_rec','Path','s_sol','t_sol','V_c_obs_circ','V_c_obs_rec')
save('data_mat/OVP_sol.mat','s_sol','t_sol','F_sol','a_sol_rec','b_sol_rec','a_sol_circ','b_sol_circ')
%% Plots

%% Fist plot
kk_convex_rec = convhull(XY_occ_obs_rec(:,1),XY_occ_obs_rec(:,2),'simplify',true);
XY_occ_obs_vertices = XY_occ_obs_rec(kk_convex_rec,:);
t_sim=(0:0.1:4).';
C_xy_simulated = repmat(V_c_obs_rec,length(t_sim),1).*repmat(t_sim,1,2)+repmat(C_xy_start_obs_rec,length(t_sim),1);
XY_occ_obs_simulated=[];
for ii=1:size(C_xy_simulated,1)
    C_xy_simulated_rep_ii = repmat(C_xy_simulated(ii,:),length(XY_occ_obs_vertices),1);
    XY_occ_obs_simulated=[XY_occ_obs_simulated;XY_occ_obs_vertices+C_xy_simulated_rep_ii;[nan nan]];
end
kk_convex_circ = convhull(XY_occ_obs_circ(:,1),XY_occ_obs_circ(:,2),'simplify',true);
XY_occ_obs_circ_vertices = XY_occ_obs_circ(kk_convex_circ,:);
t_sim=(0:0.2:8).';
C_xy_simulated = repmat(V_c_obs_circ,length(t_sim),1).*repmat(t_sim,1,2)+repmat(C_xy_start_obs_circ,length(t_sim),1);
XY_occ_obs_circ_simulated=[];
for ii=1:size(C_xy_simulated,1)
    C_xy_simulated_rep_ii = repmat(C_xy_simulated(ii,:),length(XY_occ_obs_circ_vertices),1);
    XY_occ_obs_circ_simulated=[XY_occ_obs_circ_simulated;XY_occ_obs_circ_vertices+C_xy_simulated_rep_ii;[nan nan]];
end
figureFullScreen(1)
hold on
plotRobotPath(Path,co(1,:))
plotRoboticWheelchair([0 0 0])
plotRectangle([C_xy_start_obs_rec -pi])
plotCircle([C_xy_start_obs_circ -pi/2])
plot(XY_occ_obs_simulated(:,1),XY_occ_obs_simulated(:,2),'-','Color',co(2,:));
plot(XY_occ_obs_circ_simulated(:,1),XY_occ_obs_circ_simulated(:,2),'-','Color',co(3,:));
plot(C_xy_simulated(:,1),C_xy_simulated(:,2),'-','Color',co(2,:),'LineWidth',3);
plot(C_xy_simulated(:,1),C_xy_simulated(:,2),'-','Color',co(3,:),'LineWidth',3);
% axis([-1 3 0.5 1.5])
hold off
axis equal
xlabel('x [m]')
ylabel('y [m]')
l=legend('Robot path','Footprint wheelchair','Robot start pose',...
'Fast obstalce path', 'Slow obstalce path','Location','SE');
set(l,'FontSize',30);
set(gca,'FontSize',28)
axis equal
% saveCurrentFigure('DVP_Start')


%% First and Last Time of Impact
lenObs=size(XY_occ_obs_rec,1);
C_xy_start_obs_rep_rec = repmat(C_xy_start_obs_rec,lenObs,1);
XY_occ_obs_t_start=XY_occ_obs_rec+C_xy_start_obs_rep_rec+repmat((firstImpactTime_rec)*V_c_obs_rec,lenObs,1);
XY_occ_obs_t_end=XY_occ_obs_rec+C_xy_start_obs_rep_rec+repmat((lastImpactTime_rec-2*dt)*V_c_obs_rec,lenObs,1);
c_firstImpact=C_xy_start_obs_rec+firstImpactTime_rec*V_c_obs_rec;
c_lastImpact=C_xy_start_obs_rec+(lastImpactTime_rec-2*dt)*V_c_obs_rec;
t_sim=[firstImpactTime_rec;(lastImpactTime_rec-2*dt)];
C_xy_simulated = repmat(V_c_obs_rec,length(t_sim),1).*repmat(t_sim,1,2)+repmat(C_xy_start_obs_rec,length(t_sim),1);
XY_occ_obs_simulated=[];
for ii=1:size(C_xy_simulated,1)
    C_xy_simulated_rep_ii = repmat(C_xy_simulated(ii,:),length(XY_occ_obs_vertices),1);
    XY_occ_obs_simulated=[XY_occ_obs_simulated;XY_occ_obs_vertices+C_xy_simulated_rep_ii;[nan nan]];
end
figure(2)
hold on
plotRobotPath(Path,co(1,:))
plotRoboticWheelchair([0 0 0],'k',2)
plot(XY_occ_obs_simulated(:,1),XY_occ_obs_simulated(:,2),'-','Color',co(2,:),'LineWidth',2);
% plot([XY_occ_obs_t_start(:,1);XY_occ_obs_t_end(:,1)],[XY_occ_obs_t_start(:,2);XY_occ_obs_t_end(:,2)],'.','Color',co(2,:));
axis([-1 3 0.5 1.5])
hold off
axis equal
xlabel('x [m]')
ylabel('y [m]')
l=legend('Path','Footprint wheelchair','Start pose','Obstacle','Location','NW');
text(2.15,0.5,'First Impact Time','FontSize',12)
text(0.35,1,'Last Impact Time','FontSize',12)
set(l,'FontSize',12);
set(gca,'FontSize',11)
axis equal
saveCurrentFigure('DVP_FirstLastImpactTime')


%% First and last distance at fixed time
kk_convex_rec = convhull(XY_occ_obs_rec(:,1),XY_occ_obs_rec(:,2),'simplify',true);
XY_occ_obs_vertices = XY_occ_obs_rec(kk_convex_rec,:);
t_sim=(0:0.1:4).';
C_xy_simulated = repmat(V_c_obs_rec,length(t_sim),1).*repmat(t_sim,1,2)+repmat(C_xy_start_obs_rec,length(t_sim),1);
XY_occ_obs_simulated=[];
for ii=1:size(C_xy_simulated,1)
    C_xy_simulated_rep_ii = repmat(C_xy_simulated(ii,:),length(XY_occ_obs_vertices),1);
    XY_occ_obs_simulated=[XY_occ_obs_simulated;XY_occ_obs_vertices+C_xy_simulated_rep_ii;[nan nan]];
end
t_fixed = 2;
firstBlockIdx = 98;
lastBlockIdx = 279;
firstPoseBlocked= [Path.X(firstBlockIdx)  Path.Y(firstBlockIdx)  Path.TH(firstBlockIdx)]; 
lastPoseBlocked = [Path.X(lastBlockIdx)  Path.Y(lastBlockIdx)  Path.TH(lastBlockIdx)];  
XY_occ_obs_t_fixed=XY_occ_obs_rec+C_xy_start_obs_rep_rec+repmat((t_fixed)*V_c_obs_rec,lenObs,1);
blockedPath_XY = [Path.X(firstBlockIdx:lastBlockIdx)  Path.Y(firstBlockIdx:lastBlockIdx)]; 
figure(4)
hold on
plotPath(Path,co(1,:),3)
plot(blockedPath_XY(:,1),blockedPath_XY(:,2),'Color',co(2,:),'Linewidth',3);  
plot(XY_occ_obs_simulated(:,1),XY_occ_obs_simulated(:,2),'-','Color',co(2,:))
plotRoboticWheelchair(firstPoseBlocked,'k',1.5)
plotRoboticWheelchair(lastPoseBlocked,'k',1.5)
axis([-1 3 0.5 1.5])
hold off
axis equal
xlabel('x [m]')
ylabel('y [m]')
l=legend('Free path','Blocked path','Obstacle footprint','Footprint wheelchair','Location','NW');
text(0.95,-0.25,'First Impact Distance','FontSize',12)
text(1.5,2.2,'Last Impact Distance','FontSize',12)
set(l,'FontSize',12);
set(gca,'FontSize',11)
axis equal
saveCurrentFigure('DVP_FirstLastImpactDistance')

%% Example First and last distance at fixed time
t_fixed = 1.25;
t_sim=t_fixed;
C_xy_simulated = repmat(V_c_obs_rec,length(t_sim),1).*repmat(t_sim,1,2)+repmat(C_xy_start_obs_rec,length(t_sim),1);
XY_occ_obs_simulated=[];
for ii=1:size(C_xy_simulated,1)
    C_xy_simulated_rep_ii = repmat(C_xy_simulated(ii,:),length(XY_occ_obs_vertices),1);
    XY_occ_obs_simulated=[XY_occ_obs_simulated;XY_occ_obs_vertices+C_xy_simulated_rep_ii;[nan nan]];
end
firstBlockIdx = 164;
lastBlockIdx = 279;
firstPoseBlocked= [Path.X(firstBlockIdx)  Path.Y(firstBlockIdx)  Path.TH(firstBlockIdx)]; 
lastPoseBlocked = [Path.X(lastBlockIdx)  Path.Y(lastBlockIdx)  Path.TH(lastBlockIdx)];  
XY_occ_obs_t_fixed=XY_occ_obs_rec+C_xy_start_obs_rep_rec+repmat((t_fixed)*V_c_obs_rec,lenObs,1);
blockedPath_XY = [Path.X(firstBlockIdx:lastBlockIdx)  Path.Y(firstBlockIdx:lastBlockIdx)]; 
figure(5)
hold on
plotPath(Path,co(1,:),3)
plot(blockedPath_XY(:,1),blockedPath_XY(:,2),'Color',co(2,:),'Linewidth',3);  
plot(XY_occ_obs_simulated(:,1),XY_occ_obs_simulated(:,2),'-','Color',co(2,:),'Linewidth',2)
plotRoboticWheelchair(firstPoseBlocked,'k',1.5)
plotRoboticWheelchair(lastPoseBlocked,'k',1.5)
axis([-1 3 0.5 1.5])
hold off
axis equal
xlabel('x [m]')
ylabel('y [m]')
l=legend('Free Path at t = 1.25sec','Blocked path at t = 1.25sec','Obstacle at t = 1.25sec','Footprint wheelchair','Location','NW');
text(1.475,-0.15,{'First Impact Distance','at t = 1.25sec'},'FontSize',12)
text(2.02,1.48,{'Last Impact Distance','at t = 1.25sec'},'FontSize',12)
set(l,'FontSize',12);
set(gca,'FontSize',11)
axis equal
saveCurrentFigure('DVP_FirstLastImpactDistance_FixedTime')

% % Blocked S-T map
TS_occ_convex_simple_rec=simplifyConvexHull(TS_occ_rec(ConvHull_kk_rec,:));
TS_occ_convex_simple_circ=simplifyConvexHull(TS_occ_circ(ConvHull_kk_circ,:));
figureFullScreen(6)
hold on
plot(TS_occ_rec(:,1),TS_occ_rec(:,2),'.','Color',co(2,:))
plot(TS_occ_rec(ConvHull_kk_rec,1),TS_occ_rec(ConvHull_kk_rec,2),'-','Color',co(2,:),'LineWidth',3)
plot(TS_occ_convex_simple_rec(:,1),TS_occ_convex_simple_rec(:,2),'--o','Color',co(2,:),'LineWidth',3)
plot([t_fixed, t_fixed],[0 max([Path.S])],'--k','LineWidth',2)
xlabel('time [s]')
ylabel('distance along path [m]')
axis([0 4 0 max([Path.S])])
l=legend('Occupied s-t','Convex hull','Simplified convex hull','fixed obstacle at t=1.25s','Location','SE');
set(l,'FontSize',45);
set(gca,'FontSize',43)
axis equal
saveCurrentFigure('DVP_TSocc_Diagram_Convex')

%%
TS_occ_convex_simple_rec=simplifyConvexHull(TS_occ_rec(ConvHull_kk_rec,:));
TS_occ_convex_simple_circ=simplifyConvexHull(TS_occ_circ(ConvHull_kk_circ,:));
figureFullScreen(7)
hold on
plot(TS_occ_rec(:,1),TS_occ_rec(:,2),'.','Color',co(2,:))
% plot(TS_occ_circ(ConvHull_kk_circ,1),TS_occ_circ(ConvHull_kk_circ,2),'-','Color',co(2,:),'LineWidth',3)
plot(TS_occ_convex_simple_rec(:,1),TS_occ_convex_simple_rec(:,2),'--o','Color',co(2,:),'LineWidth',3)
plot(TS_occ_circ(:,1),TS_occ_circ(:,2),'.','Color',co(3,:))
% plot(TS_occ_rec(ConvHull_kk_rec,1),TS_occ_rec(ConvHull_kk_rec,2),'-','Color',co(3,:),'LineWidth',3)
plot(TS_occ_convex_simple_circ(:,1),TS_occ_convex_simple_circ(:,2),'--o','Color',co(3,:),'LineWidth',3)
plot (t_sol, s_sol(1,:),'-','Color',co(1,:),'LineWidth',3)
plot([0 8],[s_sol(1,end) s_sol(1,end)],'Color',co(5,:),'LineWidth',6)
l=legend('Occupied s-t (fast obstacle)','Simplified convex hull','Occupied s-t (slow obstacle)','Simplified convex hull','Optimal speed for fixed path','Goal Position','Location','NE');
xlabel('Time [s]')
ylabel('distance along path [m]')
axis([0 6 0 max([Path.S])])
set(l,'FontSize',30);
set(gca,'FontSize',28)
saveCurrentFigure('DVP_Solution');

figureFullScreen(8)
hold on
plot(TS_occ_rec(:,1),TS_occ_rec(:,2),'.','Color',co(2,:))
% plot(TS_occ_circ(ConvHull_kk_circ,1),TS_occ_circ(ConvHull_kk_circ,2),'-','Color',co(2,:),'LineWidth',3)
plot(TS_occ_convex_simple_rec(:,1),TS_occ_convex_simple_rec(:,2),'--o','Color',co(2,:),'LineWidth',3)
plot(TS_occ_circ(:,1),TS_occ_circ(:,2),'.','Color',co(3,:))
% plot(TS_occ_rec(ConvHull_kk_rec,1),TS_occ_rec(ConvHull_kk_rec,2),'-','Color',co(3,:),'LineWidth',3)
plot(TS_occ_convex_simple_circ(:,1),TS_occ_convex_simple_circ(:,2),'--o','Color',co(3,:),'LineWidth',3)
plot([0 8],[s_sol(1,end) s_sol(1,end)],'Color',co(5,:),'LineWidth',6)
l=legend('Occupied s-t (fast obstacle)','Simplified convex hull','Occupied s-t (slow obstacle)','Simplified convex hull','Goal Position','Location','NE');
xlabel('Time [s]')
ylabel('distance along path [m]')
axis([0 6 0 max([Path.S])])
set(l,'FontSize',30);
set(gca,'FontSize',28)
saveCurrentFigure('DVP_Problem');