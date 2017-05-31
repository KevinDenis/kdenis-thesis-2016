%=========================================================================%
%                                                                         %
%                          Build Occupancy Grid                           %
%                          --------------------                           %
%                        (Path and Obstacle Based)                        %
%                                                                         %
% Overview :                                                              %
% * First, the traditional "Path Based" Occupancy grid is calculated,     % 
%   since this is the most logical way for generating an occupancy grid   %
%   of each path, by letting the shape of the robot move over the paths   %
%   calculated in the previous step.                                      %
%   This part updates the State Latice Structure.                         %
% * Secondly, the Obstacke Based Occupancy grid is generated using the    %
%   previously calculated data.  A clever trick is used in order to       %
%   generate his obstacle based occupancy grid in an efficient way.       %
%   This saves a lot of computing time, compared to a naive search based  %
%   method. Clever trick exmplained below, in "Obstacle Based Approach".  %
%   This part of the code will create a new structure, ObstacleTable.     %
%   A Matrix form of this structure will also be kept, XY_ObsTable        %
%   ObstacleTable contains all paths where the robot occupies a certain   %
%   x-y position and at what idx ("time") this happens.                   %
%                                                                         %
% Kevin DENIS, KU Leuven, 2016-17                                         %
% Master Thesis: Path planning algorithm for semi-autonomous              %
%                mobile robots with fast and accurate collision checking  % 
%                                                                         %
%=========================================================================%

%=========================================================================%
%                                                                         %
%                        State Latice Structure                           %
%                        ----------------------                           %
%             [ x0 y0 th0 x1 y1 th1 X Y TH S K k dk Ltot                  %
%              intK PathOccXY pathCost free idxBlocked ID]                %
%                                                                         %
%                        Obstacle Table Structure                         %
%                        ------------------------                         %
%                     [ x y (path)ID (path) blockedIdx]                   %
%                                                                         %
%=========================================================================%


%% Statup
gridRes=0.02;

%% Get Occupancy Grid of Full and Shell Robot
% Import Data from bitmap picture
[XY_occ_full] = getOccXYFromBmpRobot('RobotFull_1cm.bmp',gridRes/2);
[XY_occ_shell]= getOccXYFromBmpRobot('RobotShell_1cm.bmp',gridRes/2);

%% Create empty pathh occ grid
PathOccGridEmpty=robotics.BinaryOccupancyGrid(12,12,1/gridRes);
PathOccGridEmpty.GridLocationInWorld=[-6-gridRes/2 -6-gridRes/2];
PathOccGrid=copy(PathOccGridEmpty);
[ii_PathOccGrid,jj_Path]=meshgrid(1:PathOccGrid.GridSize(1),1:PathOccGrid.GridSize(2));
ii_all_Path=ii_PathOccGrid(:);
jj_all_Path=jj_Path(:);
%% Path based approach
progressbar('Calculating Occupancy Grid [Path Based]')


n=length(StateLattice); 
tic
for nn=1:n
    X=StateLattice(nn).X;
    Y=StateLattice(nn).Y;
    TH=StateLattice(nn).TH;
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

%     PathOccXY=unique(PathOccXY,'rows'); 
    StateLattice(nn).PathOccXY=PathOccXY; %#ok<SAGROW> % False Positive, is it not growing !
    progressbar(nn/n)
end
toc