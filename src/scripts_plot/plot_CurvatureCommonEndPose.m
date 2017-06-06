initPlotScripts
co=get(groot,'DefaultAxesColorOrder');

load('LSL_bezier.mat')
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

S_bezier = LSL_bezier(idxBezier).S;
K_bezier = LSL_bezier(idxBezier).K;

S_bezier_mod = optPath.S;
K_bezier_mod = optPath.K;

S_cloth = LSL_cloth(idxCloth).S;
K_cloth = LSL_cloth(idxCloth).K;

fig=figureFullScreen();
fig.Renderer='Painters';
subplot(1,2,1)
title('')
hold on
grid on
grid minor
plot(S_bezier,K_bezier,'LineWidth',2)
plot(S_cloth, K_cloth,'LineWidth',2)
l=legend('Curvature Bézier Curve','Curvature clothoid','Location','SW');
set(l,'FontSize',26);
set(gca,'FontSize',24)
axis([0 2.5 -0.4 1])
xlabel('s [m]')
ylabel('\kappa [m^{-1}]')

subplot(1,2,2)
title('')
hold on
grid on
grid minor
plot(S_bezier_mod,K_bezier_mod,'LineWidth',2)
plot(S_cloth, K_cloth,'LineWidth',2)
l=legend('Curvature Bézier Curve (mod obj function)','Curvature clothoid','Location','SW');
set(l,'FontSize',26);
set(gca,'FontSize',24)
axis([0 2.5 -0.4 1])
xlabel('s [m]')
ylabel('\kappa [m^{-1}]')

% saveCurrentFigure('MPCurvatureComp');