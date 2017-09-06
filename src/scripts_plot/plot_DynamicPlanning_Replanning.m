initWorkspace
co=get(groot,'DefaultAxesColorOrder');
if ~exist('LSL', 'var'); load('LSL_cloth_backup.mat'); end
if ~exist('ObstacleTable', 'var'); load('ObstacleTable_cloth.mat');  end
if ~exist('XY_ObsTable', 'var'); load('XY_ObsTable_cloth.mat'); end

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

%% Path Info and time discritisation
PathOccGrid_XY_idx=Path.PathOccXY;
PathOccGrid_XY = PathOccGrid_XY_idx(:,1:2);
dt = 2e-2/2;
t=0:dt:20;

% Dynamic Obstacle Info
XY_occ_obs_rec=getXYobsFromBMP('obs_rectangle.bmp');
C_xy_start_obs_rec = [3.5 0.75];
V_c_obs_rec = [-1 0]; 
[firstImpactTime_rec,lastImpactTime_rec]=getTimeFirstLastImpact(PathOccGrid_XY,XY_occ_obs_rec,C_xy_start_obs_rec,V_c_obs_rec,t);
t_impact_rec = firstImpactTime_rec:dt:lastImpactTime_rec;
[TS_occ_rec_1,ConvHull_kk_rec_1]=getTSOccGrid(t_impact_rec,Path,RobotHull,XY_occ_obs_rec,C_xy_start_obs_rec,V_c_obs_rec);


Path=MP_fwd(135);
verticesMatrix = [[MP_fwd.x1].' [MP_fwd.y1].'];
idx=find(ismember(verticesMatrix,[2 1.5],'rows'));
%% Path Info and time discritisation
PathOccGrid_XY_idx=Path.PathOccXY;
PathOccGrid_XY = PathOccGrid_XY_idx(:,1:2);
%Rectangle
XY_occ_obs_rec=getXYobsFromBMP('obs_rectangle.bmp');
C_xy_start_obs_rec = [3.5 0.75];
V_c_obs_rec = [-1 0]; 
[firstImpactTime_rec,lastImpactTime_rec]=getTimeFirstLastImpact(PathOccGrid_XY,XY_occ_obs_rec,C_xy_start_obs_rec,V_c_obs_rec,t);
t_impact_rec = firstImpactTime_rec:dt:lastImpactTime_rec;
[TS_occ_rec_2,ConvHull_kk_rec_2]=getTSOccGrid(t_impact_rec,Path,RobotHull,XY_occ_obs_rec,C_xy_start_obs_rec,V_c_obs_rec);

%% Fist plot

% % Blocked S-T map
TS_occ_convex_simple_rec_1=simplifyConvexHull(TS_occ_rec_1(ConvHull_kk_rec_1,:));
TS_occ_convex_simple_rec_2=simplifyConvexHull(TS_occ_rec_2(ConvHull_kk_rec_2,:));

figureFullScreen(5)
subplot(1,2,1)
hold on
plotRoboticWheelchair([0 0 0])
plotPath(MP_fwd(136),co(2,:))
plotPath(MP_fwd(135),co(4,:))
xlabel('x [m]')
ylabel('y [m]')
% l=legend('Occupied s-t','Convex hull','Simplified convex hull','Location','SE');
% set(l,'FontSize',45);
set(gca,'FontSize',43)
axis equal
% saveCurrentFigure('DVP_TSocc_Diagram_DualPath')

%%

figureFullScreen(6)
subplot(1,2,1)

hold on
% plot(TS_occ_rec_1(:,1),TS_occ_rec_1(:,2),'.','Color',co(2,:))
% plot(TS_occ_rec_2(:,1),TS_occ_rec_2(:,2),'.','Color',co(4,:))
plot(TS_occ_convex_simple_rec_1(:,1),TS_occ_convex_simple_rec_1(:,2),'-','Color',co(2,:),'LineWidth',3)
plot(TS_occ_convex_simple_rec_2(:,1),TS_occ_convex_simple_rec_2(:,2),'-','Color',co(4,:),'LineWidth',3)
% plot(TS_occ_convex_simple_rec_1(:,1),TS_occ_convex_simple_rec_1(:,2),'--o','Color',co(2,:),'LineWidth',3)
% plot(TS_occ_convex_simple_rec_2(:,1),TS_occ_convex_simple_rec_2(:,2),'--o','Color',co(2,:),'LineWidth',3)

plot([0 2.4 2.4+1.85],[0 0.95 2.78],'--','Color',co(2,:),'LineWidth',2)
plot([0 2.65 2.65+2.08],[0 0.7 2.78],'--','Color',co(4,:),'LineWidth',2)
plot([0 8],[2.78 2.78],'Color',co(5,:),'LineWidth',6)
xlabel('time [s]')
ylabel('distance along path [m]')
axis([0 5 0 2.78])
% l=legend('Occupied s-t','Convex hull','Simplified convex hull','Location','SE');
% set(l,'FontSize',45);
set(gca,'FontSize',43)
% axis equal
saveCurrentFigure('DVP_TSocc_Diagram_Replanning')


