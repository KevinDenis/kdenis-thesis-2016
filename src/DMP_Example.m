%==========================================================================
%
%              Application of LPT on Discrete Motion Planning
%              ==============================================
%
%                       Part 1. Create State Lattice
%                       ----------------------------
%   *
%   *
%   *
%
%                       Part 2. Build Occupancy Grid
%                       ----------------------------
%
%   * This code differces from BuildOccGridFromLSL because here paths
% length are not adjusted if they cause a collision, their are simply
% removed from the set (marked as blocked on the online-phase).
%   * This makes the creation of the StateLattice Structure simpler, since
% calculating at which time/length/idx a certain cel is visited for the
% first time is not nececeaaru anymore. Just knowing which cells are
% occupied by taking a certain path is needed.
%   * This updates the State Latice Structure.
%
%                          Part 3. Path Planning
%                          ---------------------
%
%   * Plan a path from the centre of the robot lab to the elevator
% The wheelchair has to enter the elevator "backwards", but should plan to
% Drive the least amount of time in reverse, this is not comfortable for
% the driver.
%   * First, a 2D planner is used, based on the Voronoi diagram of the map
% This maximises the distance to each obstacle, therefore the map is not
% inflated before the use of this 2D planner.
%   * Then, at certain distances of on that path (maxNodeDist), a State
% Lattice is drawn, indicating feasible connections in the surrounding of
% that position.
%   * Obstacle free paths (this time, not length adjusted, just removed)
% are calculated and then Dijkstra's algorithm is used to find a feasible
% path from start to end pose.
%
%
% Kevin DENIS, KU Leuven, 2016-17
% Master Thesis: Path planning algorithm for semi-autonomous
%                mobile robots with fast and accurate collision checking
%
%==========================================================================

%#ok<*UNRCH> % MATLAB, don't display warning for unreachable statements
initWorkspace

%% User settings
defaultStartEnd     = 1;
showLSL             = 1;
showVoronoiPath     = 1;
usePrecomputedData  = 1; % use precomputed data y/n --> 1/0

if usePrecomputedData
    LSLset=DMP_getLocalStateLatticeSettings();
    grid_XY=BuildMultiSizeGrid(LSLset);
    robotPose=[0 0 0];
    tic
    fprintf('Loading precomputed data ...');
    load('LSL_DMP.mat')
    fprintf(' done ! (took %2.3f sec) \n',toc)
    if showLSL
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
else
    fprintf('Calculating Local State Lattice and Path Occupancy Grid based on clothoid ... \n');
    LSLset=DMP_getLocalStateLatticeSettings();
    LSL=DMP_BuildLSLForDMP(LSLset,true);
    LSL=DMP_BuildOccGridForDMP(LSL);
    fprintf('Calculations for LSL and OG done ! \n');
end
[voronoi_path, xyth_start, xyth_dest]=DMP_Voronoi_2Dpath(defaultStartEnd,showVoronoiPath);
DMP_OptPath(LSL,voronoi_path,xyth_start,xyth_dest);

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

function LSL=DMP_BuildOccGridForDMP(LSL)
fprintf('\t Calculating path occupancy grid ... ');

tic
gridRes=0.02;

% Get Occupancy Grid of Full and Shell Robot Import Data from bitmap picture
[XY_occ_full] = getOccXYFromBmpRobot('RobotFull_1cm.bmp',gridRes/2);
[XY_occ_shell]= getOccXYFromBmpRobot('RobotShell_1cm.bmp',gridRes/2);

% Create empty pathh occ grid
PathOccGridEmpty=robotics.BinaryOccupancyGrid(12,12,1/gridRes);
PathOccGridEmpty.GridLocationInWorld=[-6-gridRes/2 -6-gridRes/2];
PathOccGrid=copy(PathOccGridEmpty);
[ii_PathOccGrid,jj_Path]=meshgrid(1:PathOccGrid.GridSize(1),1:PathOccGrid.GridSize(2));
ii_all_Path=ii_PathOccGrid(:);
jj_all_Path=jj_Path(:);

% Moving the robot's footprint over each path
progressbar('Calculating Occupancy Grid')
n=length(LSL);
tic
for nn=1:n
    X=LSL(nn).X;
    Y=LSL(nn).Y;
    TH=LSL(nn).TH;
    PathOccGrid=copy(PathOccGridEmpty); % just assigning doesn't work
    for idxPath=1:1:length(X)
        if idxPath==1 % use full occ grid (just once)
            XY_occ_kk=XY_occ_full;
        else % use shell grid (for the rest, faster ! but be sure that fine enough movement)
            XY_occ_kk=XY_occ_shell;
        end
        XY_occ_rot_trans=RotTransXY(XY_occ_kk ,TH(idxPath),X(idxPath),Y(idxPath));
        setOccupancy(PathOccGrid,XY_occ_rot_trans,ones(size(XY_occ_rot_trans,1),1))
    end
    occval_Path =getOccupancy(PathOccGrid,[ii_all_Path jj_all_Path], 'grid');
    ii_occ_Path=ii_all_Path(occval_Path==1);
    jj_occ_Path=jj_all_Path(occval_Path==1);
    PathOccXY=sortrows(grid2world(PathOccGrid,[ii_occ_Path jj_occ_Path]));
    LSL(nn).PathOccXY=PathOccXY;
    progressbar(nn/n)
end
fprintf(' done ! (took %2.3f sec) \n',toc)
end


function [voronoi_path, xyth_start, xyth_dest]=DMP_Voronoi_2Dpath(defaultStartEnd,showPlot)
[~,LabGridEdges,~] = getMapEdgeXYOccFormBmp('RobotLabo_Lift_Black.bmp',0.02);
figureFullScreen()
if defaultStartEnd
    xy_start=[5.5 8];
    xy_dest=[19.75 16.25];
    xyth_start = [xy_start pi/2];
    xyth_dest = [xy_dest pi];
else
    hold on
    show(LabGridEdges)
    title('Select start position (1st click) and orientation (2nd click)')
    xlabel('x [m]')
    ylabel('y [m]')
    set(gca,'FontSize',14)
    set(gca, 'box', 'off')
    % select start pose
    [xStart,yStart]=ginput(1);
    scatter(xStart,yStart,'*r')
    [xOrientStart,yOrientStart]=ginput(1);
    plot([xStart xOrientStart],[yStart yOrientStart],'*-r')
    thStart=atan2(yOrientStart-yStart,xOrientStart-xStart);
    xyth_start=[xStart yStart thStart];
    plotRoboticWheelchair(xyth_start);
    title('Select end position (1st click) and orientation (2nd click)')
    [xEnd,yEnd]=ginput(1);
    scatter(xEnd,yEnd,'*r')
    [xOrientEnd,yOrientEnd]=ginput(1);
    plot([xEnd xOrientEnd],[yEnd yOrientEnd],'*-r')
    thStart=atan2(yOrientEnd-yEnd,xOrientEnd-xEnd);
    xyth_dest=[xEnd yEnd thStart];
    plotRoboticWheelchair(xyth_dest);
    drawnow
    hold off
end

pathRes = 0.25;
maxNodeDist = 0.5;
xyth_start(1:2)=round(round(xyth_start(1:2)/pathRes)*pathRes,2);
xyth_start(3)=round(xyth_start(3)/pi*4)*pi/4;
xy_start = xyth_start(1:2);
xyth_dest(1:2)=round(round(xyth_dest(1:2)/pathRes)*pathRes,2);
xyth_dest(3)=round(xyth_dest(3)/pi*4)*pi/4;
xy_dest = xyth_dest(1:2);
fprintf('Start pose after discretization : [%2.2f, %2.2f, %2.2f°] \n', xyth_start(1),xyth_start(2),xyth_start(3)*180/pi);
fprintf('End pose after discretization : [%2.2f, %2.2f, %2.2f°] \n', xyth_dest(1),xyth_dest(2),xyth_dest(3)*180/pi);

clf

hold on
show(LabGridEdges)
title('Start end Pose after discritization')
plotRoboticWheelchair(xyth_start);
plotRoboticWheelchair(xyth_dest);
hold off
xlabel('x [m]')
ylabel('y [m]')
set(gca,'FontSize',14)
set(gca, 'box', 'off')
drawnow
voronoi_path= VoronoiOptimalPath(xy_start, xy_dest,pathRes,maxNodeDist,showPlot);
end

function DMP_OptPath(LSL,voronoi_path,xyth_start, xyth_dest)

[map,LabGridEdges,~] = getMapEdgeXYOccFormBmp('RobotLabo_Lift_Black.bmp',0.02);

%% State Lattice Creation on 2D paths & calculating free paths
x_sift_vec=voronoi_path(:,1);
y_sift_vec=voronoi_path(:,2);
growthFactor=length(x_sift_vec);
lengthLSL=length(LSL);
lengthGSL=growthFactor*lengthLSL;
GSL(1,1)=LSL(1);
GSL(lengthGSL,1)=LSL(end);
GSL=GSL.';
kk=1:lengthLSL;
for ii=1:growthFactor
    GSL(kk)=LSL;
    kk=kk+lengthLSL;
end

tic
fprintf('Building Global State Lattice ...')
counter=0;
kk=1:length(LSL);
for ll=1:length(x_sift_vec)
    x_shift=x_sift_vec(ll);
    y_shift=y_sift_vec(ll);
    for ii=kk
        counter=counter+1;
        PathOccXY_ii        = GSL(ii).PathOccXY + repmat([x_shift y_shift],1);
        GSL(ii).x0          = GSL(ii).x0+x_shift;
        GSL(ii).y0          = GSL(ii).y0+y_shift;
        GSL(ii).x1          = GSL(ii).x1+x_shift;
        GSL(ii).y1          = GSL(ii).y1+y_shift;
        GSL(ii).X           = GSL(ii).X+x_shift;
        GSL(ii).Y           = GSL(ii).Y+y_shift;
        GSL(ii).PathOccXY   = PathOccXY_ii;
        GSL(ii).ID          = counter;
    end
    kk=kk+length(LSL);
end
fprintf(' done ! (took %2.3f sec) \n',toc)

tic
progressbar('Calculating Collision Free Paths')
fprintf('Calculating Collision Free Paths ...')
n=length(GSL);
for ii=1:n
    PathOccXY_ii=GSL(ii).PathOccXY;
    %     PathOccXY_ii=PathOccXY_ii(:,1:2);
    % check whether path is outside world limits, blocked if true
    if any(PathOccXY_ii(:,1)<(map.XWorldLimits(1))+0.5) || ...
            any(PathOccXY_ii(:,1)>(map.XWorldLimits(2))-0.5) || ...
            any(PathOccXY_ii(:,2)<(map.YWorldLimits(1))+0.5) || ...
            any(PathOccXY_ii(:,2)>(map.YWorldLimits(2))-0.5)
        GSL(ii).free=false;
    else
        if any(getOccupancy(map,PathOccXY_ii))
            GSL(ii).free=false; % actual collision checking. No lookup table used, this would ask to big matrices
        end
    end
    progressbar(ii/n)
end
GSL_free=GSL([GSL.free].');
fprintf(' done ! (took %2.3f sec) \n',toc)

%% Graph theory with Dijkstra's algorithm
tic
fprintf('Calculating optimal path with Dijkstras algorithm ...')

% Building vertices
vx=[[GSL_free.x0].' [GSL_free.x1].']; % vertex vx = [x_start x_end]
vy=[[GSL_free.y0].' [GSL_free.y1].']; % vertex vy = [y_start y_end]
vth=[[GSL_free.th0].' [GSL_free.th1].']; % vertex vth = [th_start th_end]

% construct graph
xyth_all = unique([vx(:) vy(:) vth(:)], 'rows'); % all unique vertices
xyth_0 = [vx(:,1) vy(:,1) vth(:,1)]; % all start vertices for edge
xyth_1 = [vx(:,2) vy(:,2) vth(:,2)]; % all end vertices for edge
xyth_01= [xyth_0 xyth_1]; % start end vertices for all edges
[~,ii] = ismember(xyth_0,xyth_all,'rows'); % search idx of start idx vertix in all vertices
[~,jj] = ismember(xyth_1,xyth_all,'rows'); % search idx of end idx vertix in all vertices
pathCost=[GSL_free.pathCost].'; % since edges have the same order as the GSL, just collect all the pathCost this way
G = sparse(ii,jj,pathCost,length(xyth_all),length(xyth_all)); % create sparse matix, needed for graphshortestpath (uses Dijkstra's algorithm)
st_idx   = findrow_mex(xyth_all,xyth_start); % fast single row search
dest_idx = findrow_mex(xyth_all,xyth_dest); % fast single row search

% Dijkstra's algorithm to find shortes path
[~,opt_vertices,~]=graphshortestpath(G,st_idx,dest_idx);
xyth_opt_vertices = xyth_all(opt_vertices,:);
xyth_opt_vertices_01 = [xyth_opt_vertices(1:end-1,:) xyth_opt_vertices(2:end,:)];
[~,idx_optimal_edges] =  ismember(xyth_opt_vertices_01,xyth_01,'rows');
OptPath=GSL_free(idx_optimal_edges);
fprintf(' done ! (took %2.3f sec) \n',toc)

% plot optimal path
figureFullScreen()
show(LabGridEdges)
title('Discrete Motion Planning')
hold on
plotRobotPath(OptPath)
hold off
xlabel('x [m]')
ylabel('y [m]')
set(gca,'FontSize',14)
set(gca,'FontSize',14)
set(gca, 'box', 'off')

end