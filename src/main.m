%==========================================================================
%
%             Main Program File Local Path Planning Algorithm
%             -----------------------------------------------
%
% This file builds the local path template using a Local State Lattice and
% an obstacle based look-up table for a fast and accurate collision checking
% Several plots are shown to the user to evaluate and to inspect
% the obtained trajectories.
%
% User settings :
%   * use pre-computed data y/n (bool value usePrecomputedData)
%   * save data in data directory y/n (bool value saveCalculatedData)
%   * select curve geometry {[1],2,3} to select
%     {[Clothoid], Circular, Bézier}. (int value selectCurve)
%   * select map {[1],2,3,4} to select (int value selectMap)
%     {[RobotLab_Elevator], RobotLab, RobotLab_ZoomEntrance, Elevator}')
%   * see Local Path Planning y/n (bool value seeLPP)
%
% Overview (more information is given in each function) :
%
%   * initWorkspace : adds needed directories to path for help functions
%
%   * getLocalStateLatticeSettings : loads additional settings for the LSL
%
%   * BuildMultiSizeGrid : creates a multi-size grid based on LSLsettings
%
%   * BuildLSLWithClothoids : Build Local State Lattice based on clothoids
%
%   * BuildLSLWithCircles : Build Local State Lattice based on circle
%
%   * BuildLSLWithBezierCurve : Build LSL based on Bézier Curves
%
%   * BuildOccGridFromLSL : calculates the occupancy of the wheelchair
% going over each set of paths of the Local State Lattice. This can take
% up to 5min. Path based Occupancy grid will be converted to obstacle
% based occupancy grid (quite efficiently, ~10sec)
%
%   * BuildLSLColFree : given a map of the environment and a user-selected 
% robot pose, paths lengths are adjusted to be collision-free. This is
% also plotted. If the user desires to check whether all plotted paths
% are collision free, please uncomment plotRobotPath(LSL_W) at the bottom
% of the function, in the plotting section. This will plot the robot shape
% over all collision-free paths.
%
%   * LocalPathPlanning : OPTIONAL, set seeLPP to true to see the robot
% shape move over a selected path by the user. This mimics the role of
% the plan recognition algorithm. This can also be used to ensure that
% the path taken is obstacle free, by visual inspection.
%
% TODO :
%   * Calculate Bézier Curve LSL (only Motion Primitive at the moment)
%   * Make backups who can't be overwritten by user accidentally
%
% Kevin DENIS, KU Leuven, 2016-17
% Master Thesis: Path planning algorithm for semi-autonomous
%                mobile robots with fast and accurate collision checking
%
%==========================================================================

%==========================================================================
%
%                 Local State Lattice Structure (LSL)
%                 -----------------------------------
%             [ x0 y0 th0 x1 y1 th1 X Y TH S K k dk Ltot
%              intK PathOccXY pathCost free idxBlocked ID]
%
%                        Obstacle Table Structure
%                        ------------------------
%                     [ x y (path)ID (path)blockedIdx]
%
%==========================================================================

%#ok<*UNRCH> % MATLAB, don't display warning for unreachable statements
initWorkspace

%% User settings
usePrecomputedData = 1; % use precomputed data y/n --> 1/0
saveCalculatedData = 0; % save calculated data y/n --> 1/0
selectCurve        = 1; % Cloth = 1, Circular = 2, Bézier = 3; very slow calculation for Bézier Curve due to COP calculations ! % default use of precomputed data
selectMap          = 3; % RobotLab_Elevator=1, RobotLab=2, RobotLab_ZoomEntrance=3, Elevator=4
seeLPP             = 1; % simumate the plan recognition algorithm

stringSelectedCurve={'clothoid';'circular';'bézier'};

%% Main Program
if usePrecomputedData
    load('LSLset.mat')
    load('grid_XY.mat')
    fprintf(['loading data for ',stringSelectedCurve{selectCurve}, ' curve ...']);
    tic
    switch selectCurve
        case 1 % use clothoid curve as motion primitive
            load('LSL_cloth.mat')
            load('ObstacleTable_cloth.mat')
            load('XY_ObsTable_cloth.mat') % matrix version of [ObstacleTable.X ObstacleTable.Y], used for faster search
        case 2 % use circular arcs as motion primitive
            load('LSL_circ.mat')
            load('ObstacleTable_circ.mat')
            load('XY_ObsTable_circ.mat') % matrix version of [ObstacleTable.X ObstacleTable.Y], used for faster search
        case 3 % use Bézier Curve as motion primitive
            load('LSL_bezier.mat');
        otherwise
            disp('Please use selectCurve = {[1],2,3} to select {Clothoid, Circular, Bézier} based Motion Primitive')
            disp('default value 1 chosen')
            load('LSL_cloth.mat')
            load('ObstacleTable_cloth.mat')
            load('XY_ObsTable_cloth.mat') % matrix version of [ObstacleTable.X ObstacleTable.Y], used for faster search
    end
    fprintf(' done ! (took %2.3f sec) \n',toc)
else
    LSLset=getLocalStateLatticeSettings();
    [grid_XY,~,~,~,~]=BuildMultiSizeGrid(LSLset);
    disp(['Calculating data for ',stringSelectedCurve{selectCurve}, ' curve']);
    switch selectCurve
        case 1 % use clothoid curve as motion primitive
            [LSL,MotionPrem]=BuildLSLWithClothoids(grid_XY,LSLset);
        case 2 % use circular arcs as motion primitive
            [LSL]=BuildLSLWithCircles(LSLset);
            load('ObstacleTable_circ.mat')
            load('XY_ObsTable_circ.mat') % matrix version of [ObstacleTable.X ObstacleTable.Y], used for faster search
        case 3 % use Bézier Curve as motion primitive --> VERY SLOW due to COP for every Candidate End Pose
            load('LSL_bezier.mat');
            %LSL=BuildLSLWithBezierCurve(grid_XY,LSLset);
        otherwise
            fprintf('Wrong input. Please use selectCurve = {[1],2,3} to select {[Clothoid], Circular, Bézier} based Motion Primitive \n')
            fprintf('default value 1 chosen \n')
            [LSL,MotionPrem]=BuildLSLWithClothoids(grid_XY,LSLset);
    end
    [LSL,ObstacleTable,XY_ObsTable]=BuildOccGridFromLSL(LSL);
    fprintf(['Calculations for ', stringSelectedCurve{selectCurve}, ' curve  done ! \n']);
end
switch selectMap
    case 1 % use RobotLab_Elevator map
        map = 'RobotLaboLiftEdges.bmp';
    case 2 % use RobotLab map
        map = 'RobotLaboEdges.bmp';
    case 3 % use RobotLab_ZoomEntrance map
        map = 'RobotLaboEntranceEdges.bmp';
    case 4 % use Elevator map
        map = 'LiftEdges.bmp';
    otherwise
        fprintf('Wrong input. Please use selectMap = {[1],2,3,4} to select {[RobotLab_Elevator], RobotLab, RobotLab_ZoomEntrance, Elevator} \n')
        fprintf('default value 1 chosen \n')
        map = 'RobotLaboLiftEdges.bmp';
end
% robotPose=[4.4 2.05 -pi];
[LSL_W,robotPose,LabGrid]=BuildLSLColFree(LSL,ObstacleTable,XY_ObsTable,grid_XY,map,robotPose);
% LSL_W is in the World coordinates, dependant on the user-selected robot pose
if seeLPP; LocalPathPlanning(LSL_W,LabGrid,robotPose); end

%% Saving calculated data if desired by user
if saveCalculatedData && ~usePrecomputedData
    fprintf(['saving data for ',stringSelectedCurve{selectCurve}, ' curve ... ']);
    tic
%     save('data_mat/LSLset.mat','LSLset')
%     save('data_mat/grid_XY.mat','grid_XY')
    switch selectCurve
        case 1 % use clothoid curve as motion primitive
            save('data_mat/LSL_cloth_finest.mat','LSL')
            save('data_mat/ObstacleTable_cloth_finest.mat','ObstacleTable')
            save('data_mat/XY_ObsTable_cloth_finest.mat','XY_ObsTable')
        case 2 % use circular arcs as motion primitive
            save('data_mat/LSL_circ.mat','LSL')
            save('data_mat/ObstacleTable_circ.mat','ObstacleTable')
            save('data_mat/XY_ObsTable_circ.mat','XY_ObsTable')
        case 3 % use Bézier Curve as motion primitive
            save('data_mat/LSL_bezier.mat','LSL')
            save('data_mat/ObstacleTable_bezier.mat','ObstacleTable')
            save('data_mat/XY_ObsTable_bezier.mat','XY_ObsTable')
    end
    fprintf(' done ! (took %2.3f sec) \n',toc)
end