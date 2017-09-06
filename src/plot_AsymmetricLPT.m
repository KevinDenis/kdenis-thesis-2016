initWorkspace
co=get(groot,'DefaultAxesColorOrder');
if ~exist('LSL', 'var'); load('LSL_cloth_backup.mat'); end
load('grid_XY.mat')

LSLset=getLocalStateLatticeSettings;

robotPose = [0 0 0];



LSL_fwd=getForwardMotionFromStateLattice(LSL);
LSL_bcw=getBackwardMotionFromStateLattice(LSL);

[MP_fwd] = getMotionPrimitiveFromStateLattice(LSL_fwd);
[MP_bcw] = getMotionPrimitiveFromStateLattice(LSL_bcw);

MP_fwd_vertices = getStartEndVerticesPath(MP_fwd);
MP_bcw_vertices = getStartEndVerticesPath(MP_bcw);

LSL_vertices = getStartEndVerticesPath(LSL);


% Forward - left
idx_LSL_FwdLeft = all([LSL_vertices(:,2)>0.9,LSL_vertices(:,1)>0],2);
idx_LSL_MP = all([LSL_vertices(:,1)==0,LSL_vertices(:,2)==0,LSL_vertices(:,3)==0],2);
idx_MP_FwdLeft = MP_fwd_vertices(:,5)>0.9;
selectedIdx_FwdLeft = any([idx_LSL_FwdLeft,idx_LSL_MP],2);
[LSL_FwdLeft] =LSL(selectedIdx_FwdLeft);

% Backwards - straight
idx_LSL_BcwStraight = all([LSL_vertices(:,2)<0.9,LSL_vertices(:,2)>-0.9,LSL_vertices(:,1)<0],2);
idx_MP_BcwStraight = all([MP_bcw_vertices(:,5)<0.9,MP_bcw_vertices(:,5)>-0.9],2) ;
selectedIdx_BcwStraight = any([idx_LSL_BcwStraight,idx_LSL_MP],2);
[LSL_BcwStraight] =LSL(selectedIdx_BcwStraight);

figureFullScreen(1)
title('')
hold on
grid on
xlabel('x [m]')
ylabel('y [m]')
plotGrid(grid_XY,robotPose)
plotPath(LSL_FwdLeft)
plotRoboticWheelchair(robotPose)
plotStateLatticePoints(MP_fwd(idx_MP_FwdLeft),LSLset)
plotReachableEndPoses(LSL_FwdLeft,robotPose);
l=legend('Discrete grids','Clothoid','Robot pose','Expansion Position','Reachable end pose','Location','SW');
set(l,'FontSize',26);
set(gca,'FontSize',24)
axis equal
axis([-4.1 4.1 -3.1 3.1])
saveCurrentFigure('4_Eval_AsymmetricLPT_1');

figureFullScreen(2)
title('')
hold on
grid on
xlabel('x [m]')
ylabel('y [m]')
plotGrid(grid_XY,robotPose)
plotPath(LSL_BcwStraight)
plotRoboticWheelchair(robotPose)
plotStateLatticePoints(MP_bcw(idx_MP_BcwStraight),LSLset)
plotReachableEndPoses(LSL_BcwStraight,robotPose);
l=legend('Discrete grids','Clothoid','Robot pose','Expansion Position','Reachable end pose','Location','SW');
set(l,'FontSize',26);
set(gca,'FontSize',24)
axis equal
axis([-4.1 4.1 -3.1 3.1])
saveCurrentFigure('4_Eval_AsymmetricLPT_2');
