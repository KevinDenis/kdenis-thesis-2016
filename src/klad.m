initWorkspace
[LSLset] = DMP_getLocalStateLatticeSettings();
LSL2=DMP_BuildLSLForDMP(LSLset,0);
% State Lattice Set by generating Motion Primitive at State Lattice Positions
LSLset_mod=LSLset;
LSL=[];
LSL_ToS=[];
for th0 = wrap2Pi(0:pi/4:(2*pi-pi/4))
    LSLset.th0=th0;
    MP = getClothoidFromGrid(grid_XY,LSLset);
    LSL_tmp=MP;
    n=length(MP);
    idxSL=n;
    for ii=1:n
        x1_ii=MP(ii).x1;
        y1_ii=MP(ii).y1;
        th1_ii=MP(ii).th1;
        if rem(x1_ii,LSLset.dxEP) == 0 && rem(y1_ii,LSLset.dxEP) == 0
            LSLset_mod.x0=x1_ii;
            LSLset_mod.y0=y1_ii;
            LSLset_mod.th0=th1_ii;
            [MP_RotTrans] = getClothoidFromGrid(grid_XY,LSLset_mod);
            idxSL=idxSL(end)+(1:length(MP_RotTrans));
            LSL_tmp(idxSL)=MP_RotTrans;
        end
    end
    LSL=[LSL;LSL_tmp];
    LSL_ToS = [LSL_ToS;getMotionPremTurnOnSpot(LSL_tmp,th0)];
end

LSL=[LSL;LSL_ToS];
LSL=[LSL;AddReverseDirection(LSL)];
LSL = CleanupLSL(LSL);
LSL = FreeAllPaths(LSL);

vertices=getStartEndVerticesPath(LSL);
vertices2=getStartEndVerticesPath(LSL2);

idx1=~ismember(vertices,vertices2,'rows');
vertices(idx1,:)
idx2=~ismember(vertices2,vertices,'rows');
vertices2(idx2,:)

    figure()
    title('')
    hold on
    grid on
    xlabel('x [m]')
    ylabel('y [m]')
    axis equal
    plotGrid(grid_XY,robotPose)
    plotPath(LSL2)
    plotSimpleRobot(robotPose)
    l=legend('Discrete grids','Clothoids','Robot pose','Location','SE');
    set(l,'FontSize',16);
    set(gca,'FontSize',14)
    hold off

function [LSLset] = DMP_getLocalStateLatticeSettings()
res=1e-2; % extra safty factor of 2
LSLset.x0=0; 
LSLset.y0=0;
LSLset.th0=0;
LSLset.xmax=2;
LSLset.ymax=2;
LSLset.kmax=1;
LSLset.res = res;
LSLset.dth = pi/4;
LSLset.dxEP=10;

% grid 1
LSLset.dx1=0.25;
LSLset.x1_max=0.5;
% grid 2
LSLset.dx2=0.25;
LSLset.x2_max=1;
% grid 3
LSLset.dx3=0.25;
LSLset.x3_max=2;
LSLset.y3_max=2;

LSLset.ROI=[res LSLset.ymax; LSLset.xmax LSLset.ymax; LSLset.xmax -LSLset.ymax;res -LSLset.ymax;res LSLset.ymax];
end


function LSL=DMP_BuildLSLForDMP(LSLset,showPlot)

grid_XY=BuildMultiSizeGrid(LSLset);
robotPose=[0 0 0];
LSLset_mod=LSLset;

%% Motion Primitive from [0 0 0], [0 0 45°] and [0 0 -45°]
[MP_0deg] = getClothoidFromGrid(grid_XY,LSLset);
LSLset_mod.th0=pi/4;
MP_45deg=getClothoidFromGrid(grid_XY,LSLset_mod);
LSLset_mod.th0=-pi/4;
MP_45degMin=getClothoidFromGrid(grid_XY,LSLset_mod);

%% Local State Lattice from Motion Primitive at [0 0 0]
LSL_0deg=MP_0deg;
n=length(MP_0deg);
idxEP=n;
for ii=1:n
    x1_ii=MP_0deg(ii).x1;
    y1_ii=MP_0deg(ii).y1;
    th1_ii=MP_0deg(ii).th1;
    if rem(x1_ii,LSLset.dxEP) == 0 && rem(y1_ii,LSLset.dxEP) == 0
        LSLset_mod.x0=x1_ii;
        LSLset_mod.y0=y1_ii;
        LSLset_mod.th0=th1_ii;
        [MP_RotTrans] = getClothoidFromGrid(grid_XY,LSLset_mod);
        idxEP=idxEP(end)+(1:length(MP_RotTrans));
        LSL_0deg(idxEP)=MP_RotTrans;
    end
end

%% Local State Lattice from Motion Primitive at [0 0 45°]
LSL_45deg=MP_45deg;
n=length(MP_45deg);
idxEP=n;
for ii=1:n
    x1_ii=MP_45deg(ii).x1;
    y1_ii=MP_45deg(ii).y1;
    th1_ii=MP_45deg(ii).th1;
    if rem(x1_ii,LSLset.dxEP) == 0 && rem(y1_ii,LSLset.dxEP) == 0
        LSLset_mod.x0=x1_ii;
        LSLset_mod.y0=y1_ii;
        LSLset_mod.th0=th1_ii;
        [MP_RotTrans] = getClothoidFromGrid(grid_XY,LSLset_mod);
        idxEP=idxEP(end)+(1:length(MP_RotTrans));
        LSL_45deg(idxEP)=MP_RotTrans;
    end
end

%% Local State Lattice from Motion Primitive at [0 0 -45°]
LSL_45degMin=MP_45degMin;
n=length(MP_45degMin);
idxEP=n;
for ii=1:n
    x1_ii=MP_45degMin(ii).x1;
    y1_ii=MP_45degMin(ii).y1;
    th1_ii=MP_45degMin(ii).th1;
    if rem(x1_ii,LSLset.dxEP) == 0 && rem(y1_ii,LSLset.dxEP) == 0
        LSLset_mod.x0=x1_ii;
        LSLset_mod.y0=y1_ii;
        LSLset_mod.th0=th1_ii;
        [MP_RotTrans] = getClothoidFromGrid(grid_XY,LSLset_mod);
        idxEP=idxEP(end)+(1:length(MP_RotTrans));
        LSL_45degMin(idxEP)=MP_RotTrans;
    end
end

LSL=[LSL_45degMin;LSL_0deg;LSL_45deg];

% Rotate the combined LSL at 90 -90 180
LSL_90deg=RotTransMotionPrem(LSL,pi/2,0,0);
LSL_90degMin=RotTransMotionPrem(LSL,-pi/2,0,0);
LSL_180deg=RotTransMotionPrem(LSL,pi,0,0);

LSL=[LSL;LSL_90deg;LSL_90degMin;LSL_180deg];

SL_ToS=[];
for th0=(-3/4*pi:pi/4:pi) 
    SL_ToS = [SL_ToS;getMotionPremTurnOnSpot(LSL,th0)];
end

LSL=[LSL;SL_ToS];
LSL=[LSL;AddReverseDirection(LSL)];
LSL = CleanupLSL(LSL);
LSL = FreeAllPaths(LSL);

%% Plot
if showPlot
    figure()
    title('')
    hold on
    grid on
    xlabel('x [m]')
    ylabel('y [m]')
    axis equal
    plotGrid(grid_XY,robotPose)
    plotPath(LSL)
    plotSimpleRobot(robotPose)
    l=legend('Discrete grids','Clothoids','Robot pose','Location','SE');
    set(l,'FontSize',16);
    set(gca,'FontSize',14)
    hold off
end
end