 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%                 Build Local State Lattice Collision Free                %
%                 ----------------------------------------                %
%                                                                         %
% Overview :                                                              %
%   * The user selects the start position of the robot followed by        %
%     it's ordientation                                                   %
%   * Given the user-selected map, and the user-selected robot pose, the  %
%   map is translated in the robot coordinate frame {R}. This because the %
%   Local Path Template / ObstacleTable / Local State Lattice is          %
%   formulated according to this Local Reference Frame {R}, which asumes  %
%   that the robot is at the orogin [0 0 0�].                             %
%   * Clossiion free trajectories are calculated efficiently and accuratly%
%   by using the pre-cumputed obstacle table, which indicates for each    %
%   dircrete grid which paths goes through it at which length. The online %
%   fase only has to mach occupied cells with the obstacle table, and     %
%   addjust each path length(only if this results in a smaller path       %
%   * Once evevry path is update, the paths are translated in the World   %
%   Coordinates {W} for a better readability for the user (LSL_W)         %
%                                                                         %
% Kevin DENIS, KU Leuven, 2016-17                                         %
% Master Thesis: Path planning algorithm for semi-autonomous              %
%                mobile robots with fast and accurate collision checking  %
%                                                                         %
%=========================================================================%

%=========================================================================%
%                                                                         %
%                 Local State Lattice Structure (LSL)                     %
%                 -----------------------------------                     %
%             [ x0 y0 th0 x1 y1 th1 X Y TH S K k dk Ltot                  %
%              intK PathOccXY pathCost free idxBlocked ID]                %
%                                                                         %
%                        Obstacle Table Structure                         %
%                        ------------------------                         %
%                     [ x y (path)ID (path)blockedIdx]                    %
%                                                                         %
%=========================================================================%

function [LSL_W,robotPose,LabGrid]=BuildLSLColFree(StateLattice,ObstacleTable,XY_ObsTable,grid_XY,bmp,varargin)
showPlot=1;
StateLattice=FreeAllPaths(StateLattice);
[LabGrid,~,XY_occ_lab] = getMapEdgeXYOccFormBmp(bmp,0.02); % grid cels occupancy grid are 0.02 --> 2cm which is 2x path resolution

%% User defined start position

if nargin == 5
    figureFullScreen()
    hold on
    show(LabGrid)
    title('Select begin position (1st click) and orientation (2nd click)')
    set(gca,'FontSize',14)
    set(gca, 'box', 'off')
    [xStart,yStart]=ginput(1);
    scatter(xStart,yStart,'*r')
    [xOrient,yOrient]=ginput(1);
    plot([xStart xOrient],[yStart yOrient],'*-r')
    thStart=atan2(yOrient-yStart,xOrient-xStart);
    robotPose=[xStart yStart thStart];
    disp(['Use defined robot pose : [', num2str(robotPose(1),3),' ',num2str(robotPose(2),3),' ',num2str(robotPose(3)*180/pi,3),'�]' ])
    close all
else
    robotPose=(varargin{1});
end


%% Transform Lab Occupancy Grid to Robot Coordinates
XY_occ_lab_R=TransRotXY(XY_occ_lab,-robotPose(3),-robotPose(1),-robotPose(2));
XY_occ_lab_R(abs(XY_occ_lab_R(:,1))>6,:)=[];
XY_occ_lab_R(abs(XY_occ_lab_R(:,2))>6,:)=[];

LabGrid_R = robotics.BinaryOccupancyGrid(12,12,50);
LabGrid_R.GridLocationInWorld=[-6 -6];
setOccupancy(LabGrid_R,XY_occ_lab_R,ones(size(XY_occ_lab_R,1),1))
% inflate(LabGrid_R,0.01)
[ii_lab_R,jj_lab_R]=meshgrid(1:LabGrid_R.GridSize(1),1:LabGrid_R.GridSize(2));
ii_all_lab_R=ii_lab_R(:);
jj_all_lab_R=jj_lab_R(:);
occval_lab =getOccupancy(LabGrid_R,[ii_all_lab_R jj_all_lab_R], 'grid');
ii_occ_lab=ii_all_lab_R(occval_lab==1);
jj_occ_lab=jj_all_lab_R(occval_lab==1);
XY_occ_lab_R=grid2world(LabGrid_R,[ii_occ_lab jj_occ_lab]);
XY_occ_lab_R=unique(round(round(XY_occ_lab_R./0.02)*0.02,2),'rows');
XY_occ_lab_R=sortrows(XY_occ_lab_R);

tic
idxFound=find(ismember(XY_ObsTable,XY_occ_lab_R,'rows'));
% idxFound=findVector(XY_ObsTable,XY_occ_lab_R);


for ii=1:length(idxFound)
    idxRow=idxFound(ii);
    IDOccPaths = [ObstacleTable(idxRow).ID];
    IdxOccPaths = [ObstacleTable(idxRow).Idx];
    for jj=1:length(IDOccPaths)
        StateLattice(IDOccPaths(jj)).free=false;
        if StateLattice(IDOccPaths(jj)).idxBlocked > IdxOccPaths(jj)
            StateLattice(IDOccPaths(jj)).idxBlocked=IdxOccPaths(jj);
        end
    end
end

disp(['Adjustment of paths length to generate collision-free trajectories took ', num2str(toc,5),' sec'])

[StateLattice] = CleanupLooseStarts(StateLattice);
% StateLattice=getForwardMotionFromStateLattice(StateLattice);
LSL_W=RotTransMotionPrem(StateLattice,robotPose(3),robotPose(1),robotPose(2));

%% Plot
if showPlot
fig=figureFullScreen();
fig.Renderer='Painters';
show(LabGrid)
title('Collision-free trajectories obtained by adjusting their lentgth using a lookup table')
hold on
grid on
xlabel('x [m]')
ylabel('y [m]')
axis equal
title('')
plotGrid(grid_XY,robotPose)
plotPath(LSL_W)
% plotRobotPath(LSL_W)
plotRoboticWheelchair(robotPose)
l=legend('Discrete Grid','Collision-free paths','Robot pose','Location','SE');
set(l,'FontSize',16);
set(gca,'FontSize',14)
set(gca, 'box', 'off')
%     pause()
%     saveCurrentFigure   

end
end