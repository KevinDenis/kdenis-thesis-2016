initPlotScripts
co=get(groot,'DefaultAxesColorOrder');

load('LSL_bezier_backup.mat')
LSL_bezier=LSL;
LSL_bezier=getMotionPrimitiveFromStateLattice(LSL_bezier);
LSL_bezier = CleanupLSL(LSL_bezier);

load('LSL_cloth.mat')
LSL_cloth=LSL;
LSL_cloth=getMotionPrimitiveFromStateLattice(LSL_cloth);
LSL_cloth = CleanupLSL(LSL_cloth);

vertices_cloth=getStartEndVerticesPath(LSL_cloth);
vertices_bezier=getStartEndVerticesPath(LSL_bezier);

poseStart=[0 0 0];
poseEnd=[1.25 1.5 pi/2];
poseStartEnd=[poseStart poseEnd];

idxBezier=findrow_mex(vertices_bezier,poseStartEnd);
idxCloth=findrow_mex(vertices_cloth,poseStartEnd);

optPath=BezierCOP(struct('t',linspace(0,1,1001),'n',4,'P0',poseStart,'P1',poseEnd,'kappa_max',1));

Path_bezier = LSL_bezier(idxBezier);
XY_bezier = [[LSL_bezier(idxBezier).X],[LSL_bezier(idxBezier).Y]];
S_bezier = LSL_bezier(idxBezier).S;
K_bezier = LSL_bezier(idxBezier).K;

Path_bezier_mod = optPath;
XY_bezier_mod = [[optPath.X],[optPath.Y]];
S_bezier_mod = optPath.S;
K_bezier_mod = optPath.K;

Path_cloth = LSL_cloth(idxCloth);
XY_cloth = [[LSL_cloth(idxCloth).X],[LSL_cloth(idxCloth).Y]];
S_cloth = LSL_cloth(idxCloth).S;
K_cloth = LSL_cloth(idxCloth).K;

% fig=figureFullScreen();
% fig.Renderer='Painters';
% subplot(1,2,1)
% title('')
% hold on
% grid on
% grid minor
% plot(S_bezier,K_bezier,'LineWidth',2)
% plot(S_cloth, K_cloth,'LineWidth',2)
% l=legend('Curvature Bézier Curve','Curvature Clothoid','Location','SW');
% set(l,'FontSize',26);
% set(gca,'FontSize',24)
% axis([0 2.5 -0.4 1])
% xlabel('s [m]')
% ylabel('\kappa [m^{-1}]')
% 
% subplot(1,2,2)
% title('')
% hold on
% grid on
% grid minor
% plot(S_bezier_mod,K_bezier_mod,'LineWidth',2)
% plot(S_cloth, K_cloth,'LineWidth',2)
% l=legend('Curvature Bézier Curve (mod obj function)','Curvature Clothoid','Location','SW');
% set(l,'FontSize',26);
% set(gca,'FontSize',24)
% axis([0 2.5 -0.4 1])
% xlabel('s [m]')
% ylabel('\kappa [m^{-1}]')

% saveCurrentFigure('MPCurvatureComp');


%% Extra figures
[LSLset] = getLocalStateLatticeSettings();
[grid_XY,ROI0,idxIn0,~,~]=BuildMultiSizeGrid(LSLset);

figureFullScreen()
title('')
hold on
xlabel('x [m]')
ylabel('y [m]')
plotPath(Path_bezier,co(1,:),3)
plotPath(Path_cloth,co(2,:),3)
plotRoboticWheelchair(robotPose)
l=legend('Bézier curve','Clothoid','Location','SE');
set(l,'FontSize',26);
set(gca,'FontSize',24)
axis equal
axis([-1 1.5 -0.5 1.75])
saveCurrentFigure('3_Design_1_LSL_MPCommonEndPose');

figureFullScreen();
subplot(1,2,1)
title('')
hold on
plot(S_bezier,K_bezier,'LineWidth',2)
plot(S_cloth, K_cloth,'LineWidth',2)
l=legend('Curvature Bézier curve','Curvature Clothoid','Location','SW');
set(l,'FontSize',26);
set(gca,'FontSize',24)
axis([0 2.25 -0.4 1])
xlabel('s [m]')
ylabel('\kappa [m^{-1}]')
saveCurrentFigure('3_Design_1_LSL_MPCommonEndPose_curv');

figureFullScreen()
hold on
title('')
xlabel('x [m]')
ylabel('y [m]')
plotPath(Path_bezier_mod,co(1,:),3)
plotPath(Path_cloth,co(2,:),3)
plotRoboticWheelchair(robotPose)
l=legend('Bézier curve','Clothoid','Location','SE');
set(l,'FontSize',26);
set(gca,'FontSize',24)
axis equal
axis([-1 1.5 -0.5 1.75])
saveCurrentFigure('3_Design_1_LSL_MPCommonEndPose_mod');

figureFullScreen()
subplot(1,2,1)
title('')
hold on
plot(S_bezier_mod,K_bezier_mod,'LineWidth',2)
plot(S_cloth, K_cloth,'LineWidth',2)
l=legend('Curvature Bézier Curve (mod obj function)','Curvature Clothoid','Location','SW');
set(l,'FontSize',26);
set(gca,'FontSize',24)
axis([0 2.25 -0.4 1])
xlabel('s [m]')
ylabel('\kappa [m^{-1}]')
saveCurrentFigure('3_Design_1_LSL_MPCommonEndPose_mod_curv');