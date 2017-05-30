initWorkspace
co=get(groot,'DefaultAxesColorOrder');

load('SL_bezier.mat')
SL_bezier=StateLattice;
SL_bezier=getMotionPrimitiveFromStateLattice(SL_bezier);
SL_bezier = CleanupStateLattice(SL_bezier);

load('SL_cloth.mat')
SL_cloth=StateLattice;
SL_cloth=getMotionPrimitiveFromStateLattice(SL_cloth);
SL_cloth = CleanupStateLattice(SL_cloth);


voxel_cloth=[[SL_cloth.x0].',[SL_cloth.y0].',[SL_cloth.th0].',[SL_cloth.x1].',[SL_cloth.y1].',[SL_cloth.th1].'];
voxel_bezier=[[SL_bezier.x0].',[SL_bezier.y0].',[SL_bezier.th0].',[SL_bezier.x1].',[SL_bezier.y1].',[SL_bezier.th1].'];

poseStart=[0 0 0];
poseEnd=[1.25 1.5 pi/2];
poseStartEnd=[poseStart poseEnd];

idxBezier=findrow_mex(voxel_bezier,poseStartEnd);
idxCloth=findrow_mex(voxel_cloth,poseStartEnd);

optPath=BezierCOP(struct('t',linspace(0,1,1001),'n',4,'P0',poseStart,'P1',poseEnd,'kappa_max',1));

S_bezier = SL_bezier(idxBezier).S;
K_bezier = SL_bezier(idxBezier).K;

S_bezier_mod = optPath.S;
K_bezier_mod = optPath.K;

S_cloth = SL_cloth(idxCloth).S;
K_cloth = SL_cloth(idxCloth).K;

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

saveCurrentFigure('MPCurvatureComp');