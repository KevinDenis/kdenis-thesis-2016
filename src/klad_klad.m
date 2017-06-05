

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





showPlot=true;
grid_XY=BuildMultiSizeGrid(LSLset);
robotPose=[0 0 0];




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
%             if x1_ii==1 && y1_ii == 1 && th1_ii==pi/2
%                 keyboard()
%             end
        end
    end
    LSL=[LSL;LSL_tmp];
%     LSL_ToS = [LSL_ToS;getMotionPremTurnOnSpot(LSL,th0)];
end


LSL=[LSL;LSL_ToS];
LSL=[LSL; StartEndSwap(LSL)];
LSL=[LSL;AddReverseDirection(LSL)];
length(LSL)
LSL = CleanupLSL(LSL);
length(LSL)
LSL = FreeAllPaths(LSL);

%% Plot
if showPlot
    figure(1)
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
end