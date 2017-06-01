%=========================================================================%
%                                                                         %
%              Application of LPT on Discrete Motion Planning             %
%              ----------------------------------------------             %
%                       Part 1. Create State Lattice                      %
%                                                                         %
%                                                                         %
% Overview :                                                              %
%   *                                                                     %
%   *                                                                     %
%   *                                                                     %
%                                                                         %
% Kevin DENIS, KU Leuven, 2016-17                                         %
% Master Thesis: Path planning algorithm for semi-autonomous              %
%                mobile robots with fast and accurate collision checking  %    
%                                                                         %
%=========================================================================%

%% Statup
initWorkspace
LSLset=getLocalStateLatticeSettings();

LSLset.xmax=2;
LSLset.ymax=1;
LSLset.dth = pi/4;
LSLset.dx1=0.25;
LSLset.x1_max=0.5;

LSLset.dx2=0.25;
LSLset.x2_max=1;

LSLset.dx3=0.25;
LSLset.x3_max=2;
LSLset.y3_max=2;

res=LSLset.res;
LSLset.ROI=[res LSLset.ymax; LSLset.xmax LSLset.ymax; LSLset.xmax -LSLset.ymax;res -LSLset.ymax;res LSLset.ymax];

grid_XY=BuildMultiSizeGrid(LSLset);
showPlot=true;
robotPose=[0 0 0];
LSLset_mod=LSLset;

dth_rot=pi/4;

%% Motion Primitive from [0 0 0]
[MotionPrem_0deg] = getClothoidFromGrid(grid_XY,LSLset);
LSLset_mod.th0=pi/4;
MotionPrem_45deg=getClothoidFromGrid(grid_XY,LSLset_mod);
LSLset_mod.th0=-pi/4;
MotionPrem_45degMin=getClothoidFromGrid(grid_XY,LSLset_mod);


%% State Lattice Set by generating Motion Primitive at State Lattice Positions
StateLattice_0deg=MotionPrem_0deg;
n=length(MotionPrem_0deg);
idxSL=n;
for ii=1:n
    x1_ii=MotionPrem_0deg(ii).x1;
    y1_ii=MotionPrem_0deg(ii).y1;
    th1_ii=MotionPrem_0deg(ii).th1;
    if rem(x1_ii,LSLset.dxEP) == 0 && rem(y1_ii,LSLset.dxEP) == 0
        LSLset_mod.x0=x1_ii;
        LSLset_mod.y0=y1_ii;
        LSLset_mod.th0=th1_ii;
        [MotionPremRotTrans] = getClothoidFromGrid(grid_XY,LSLset_mod);
        idxSL=idxSL(end)+(1:length(MotionPremRotTrans));
        StateLattice_0deg(idxSL)=MotionPremRotTrans;
    end
end

%% State Lattice Set by generating Motion Primitive at State Lattice Positions
StateLattice_45deg=MotionPrem_45deg;
n=length(MotionPrem_45deg);
idxSL=n;
for ii=1:n
    x1_ii=MotionPrem_45deg(ii).x1;
    y1_ii=MotionPrem_45deg(ii).y1;
    th1_ii=MotionPrem_45deg(ii).th1;
    if rem(x1_ii,LSLset.dxEP) == 0 && rem(y1_ii,LSLset.dxEP) == 0
        LSLset_mod.x0=x1_ii;
        LSLset_mod.y0=y1_ii;
        LSLset_mod.th0=th1_ii;
        [MotionPremRotTrans] = getClothoidFromGrid(grid_XY,LSLset_mod);
        idxSL=idxSL(end)+(1:length(MotionPremRotTrans));
        StateLattice_45deg(idxSL)=MotionPremRotTrans;
    end
end

%% State Lattice Set by generating Motion Primitive at State Lattice Positions
StateLattice_45degMin=MotionPrem_45degMin;
n=length(MotionPrem_45degMin);
idxSL=n;
for ii=1:n
    x1_ii=MotionPrem_45degMin(ii).x1;
    y1_ii=MotionPrem_45degMin(ii).y1;
    th1_ii=MotionPrem_45degMin(ii).th1;
    if rem(x1_ii,LSLset.dxEP) == 0 && rem(y1_ii,LSLset.dxEP) == 0
        LSLset_mod.x0=x1_ii;
        LSLset_mod.y0=y1_ii;
        LSLset_mod.th0=th1_ii;
        [MotionPremRotTrans] = getClothoidFromGrid(grid_XY,LSLset_mod);
        idxSL=idxSL(end)+(1:length(MotionPremRotTrans));
        StateLattice_45degMin(idxSL)=MotionPremRotTrans;
    end
end

StateLattice=[StateLattice_45degMin;StateLattice_0deg;StateLattice_45deg];

StateLattice_90deg=RotTransMotionPrem(StateLattice,pi/2,0,0);
StateLattice_90degMin=RotTransMotionPrem(StateLattice,-pi/2,0,0);
StateLattice_180deg=RotTransMotionPrem(StateLattice,pi,0,0);

StateLattice=[StateLattice;StateLattice_90deg;StateLattice_90degMin;StateLattice_180deg];

SL_ToS=[];
for th0=(-3/4*pi:pi/4:pi) 
    SL_ToS = [SL_ToS;getMotionPremTurnOnSpot(StateLattice,th0)];
end

StateLattice=[StateLattice;SL_ToS];
StateLattice=[StateLattice; StartEndSwap(StateLattice)];
StateLattice=[StateLattice;AddReverseDirection(StateLattice)];
StateLattice = CleanupLSL(StateLattice);
StateLattice = FreeAllPaths(StateLattice);

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
    plotPath(StateLattice)
    plotSimpleRobot(robotPose)
    l=legend('Discrete grids','Clothoids','Robot pose','Location','SE');
    set(l,'FontSize',16);
    set(gca,'FontSize',14)
end