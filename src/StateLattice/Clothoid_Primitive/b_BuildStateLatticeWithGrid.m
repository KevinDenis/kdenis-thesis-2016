% Notes :
%   *
%   *

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%                        State Latice Structure                           %
%[ x0 y0 th0 x1 y1 th1 X Y TH k dk Ltot intK PathOccXY pathCost free ID ] %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clean-up MATLAB
initWorkspace

%% Init
showPlot=true;
robotPose=[0 0 0];

[MotionPrem] = getMotionPremFromGrid(grid_XY,LSLset);
LSLset_mod=LSLset;
StateLattice=MotionPrem;
n=length(MotionPrem);
kk=n;
for ii=1:n
    x1_ii=MotionPrem(ii).x1;
    y1_ii=MotionPrem(ii).y1;
    th1_ii=MotionPrem(ii).th1;
    if rem(x1_ii,LSLset.dxSL) == 0 && rem(y1_ii,LSLset.dxSL) == 0
        LSLset_mod.x0=x1_ii;
        LSLset_mod.y0=y1_ii;
        LSLset_mod.th0=th1_ii;
        [MotionPremRotTrans] = getMotionPremFromGrid(grid_XY,LSLset_mod);
        kk=kk(end)+(1:length(MotionPremRotTrans));
        StateLattice(kk)=MotionPremRotTrans;
    end
    % this is used for plotting example
    if IsNear(x1_ii,1.5) && IsNear(y1_ii,1) && IsNear(th1_ii,1.1781,1e-2)
        MotionPremKeep=MotionPremRotTrans;
    end
end
StateLattice=CleanupStateLattice(StateLattice);

%% Plot Local State Lattice
if showPlot
    %% Motion Primitive 
    figure()
    title('Local State Lattice')
    hold on
    grid on
    xlabel('x [m]')
    ylabel('y [m]')
    axis equal
    plotGrid(grid_XY,robotPose)
    plot(ROI0(:,1),ROI0(:,2),'go-','Linewidth',2)
    plot(grid_XY(idxIn0,1),grid_XY(idxIn0,2),'og')
    plotPath(MotionPrem)
    plotSimpleRobot(robotPose)
    l=legend('discrete grids','ROI','Grids In ROI','clothoids','circular arcs','reachable grids','robot pose');
    set(l,'FontSize',12);
%     pause()
%     saveCurrentFigure   

    %% Example of State Lattice Point [1.5 0 5*pi/8]
    figure()
    title('Local State Lattice')
    hold on
    grid on
    xlabel('x [m]')
    ylabel('y [m]')
    axis equal
    plotGrid(grid_XY,robotPose)
    plot(ROI1(:,1),ROI1(:,2),'ro-','Linewidth',2)
    plot(grid_XY(idxIn1,1),grid_XY(idxIn1,2),'ro')
    plotPath([MotionPrem;MotionPremKeep])
    plotSimpleRobot([1.5 1 1.1781])
    l=legend('discrete grids','ROI','Grids In ROI','clothoids','circular arcs','reachable grids','robot pose');
    set(l,'FontSize',12);
%     pause()
%     saveCurrentFigure   
    
    %% Complete Set Of Paths Of State Lattice
    figure()
    title('Local State Lattice')
    hold on
    grid on
    xlabel('x [m]')
    ylabel('y [m]')
    axis equal
    plotGrid(grid_XY,robotPose)
    plotPath(StateLattice)
    plotSimpleRobot(robotPose)
    plotStateLatticePoints(MotionPrem,LSLset)
    l=legend('discrete grids','clothoids','circular arcs','reachable grids','robot pose','State Lattice Positions');
    set(l,'FontSize',12);
%     pause()
%     saveCurrentFigure
end