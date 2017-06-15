%==========================================================================
%
%                       Interactive Tool : Path Viewer
%                       ------------------------------
%
% Interactive path viewing tool
%
% User settings :
%   * select curve geometry {[1],2,3} to select
%     {[Clothoid], Circular, Bézier}. (int value selectCurve)
%   * see Robot Shape on paths y/n (bool value seeShape)
%
% Overview :
%   * WIP
%
% Kevin DENIS, KU Leuven, 2016-17
% Master Thesis: Path planning algorithm for semi-autonomous
%                mobile robots with fast and accurate collision checking
%
%==========================================================================

%% Init()
initWorkspace

%% User settings
selectCurve = 1; % Cloth = 1, Circular = 2, Bézier = 3;
seeShape    = 0; % y/n --> 1/0
showObstacleInfluance = 0; % y/n --> 1/0. Hard code commented out
stringSelectedCurve={'clothoid';'circular';'bézier'};


%% Main Program
load('LSLset.mat')
load('grid_XY.mat')
fprintf(['loading data for ',stringSelectedCurve{selectCurve}, ' curve']);
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
fprintf('... done \n');


XY_ObsGrid=[];
LSL=FreeAllPaths(LSL);
UpdatePlot(XY_ObsGrid,LSL,seeShape)

%% Main()
while 1
    XY_ObsGrid=UpdateEnviroment(XY_ObsGrid);
    LSL=UpdateStateLattice(XY_ObsGrid,LSL,ObstacleTable,XY_ObsTable,showObstacleInfluance);
    UpdatePlot(XY_ObsGrid,LSL,seeShape)
end

%% Used Functions
function LSL=UpdateStateLattice(XY_ObsGrid,LSL,ObstacleTable,XY_Table,showObstacleInfluance)

% stringaffected=[];
tic
xy_obs_end=XY_ObsGrid(end,:);
idxRow=findrow_mex(XY_Table,xy_obs_end);
IDOccPaths=0;
if ~isnan(idxRow)
    IDOccPaths = [ObstacleTable(idxRow).ID];
    IdxOccPaths = [ObstacleTable(idxRow).Idx];
    for jj=1:length(IDOccPaths)
        LSL(IDOccPaths(jj)).free=false;
        if isempty(LSL(IDOccPaths(jj)).idxBlocked) || LSL(IDOccPaths(jj)).idxBlocked > IdxOccPaths(jj)
            LSL(IDOccPaths(jj)).idxBlocked=IdxOccPaths(jj);
            %             stringaffected=[stringaffected,num2str(IDOccPaths(jj)),'(',num2str(IdxOccPaths(jj)),') '];
        end
    end
    %     dispText=['obstacle ',num2str(size(XY_ObsGrid,1)), ' affects pathID(pathIdx) ', stringaffected];
    %     if showObstacleInfluance; disp(dispText); end
end
fprintf('Length adjustment %d paths took %2.3f msec\n',length(IDOccPaths),toc*1000)
LSL=CleanupLooseStarts(LSL);
end

function XY_ObsGrid=UpdateEnviroment(XY_ObsGrid)
dx=0.02;
[x_obs,y_obs]=ginput2(1);
x_obs=round(round(x_obs/dx)*dx,2);
y_obs=round(round(y_obs/dx)*dx,2);
XY_ObsGrid=[XY_ObsGrid;x_obs y_obs];
end

function UpdatePlot(XY_ObsGrid,StateLattice,seeShape)
SL_Plot=StateLattice;
for ii=1:length(StateLattice)
    if ~SL_Plot(ii).free
        idxPlot=SL_Plot(ii).idxBlocked;
        SL_Plot(ii).X=SL_Plot(ii).X(1:idxPlot-1);
        SL_Plot(ii).Y=SL_Plot(ii).Y(1:idxPlot-1);
        SL_Plot(ii).TH=SL_Plot(ii).TH(1:idxPlot-1);
    end
end

figure(1)
clf
title('Local State Lattice (press RMB for zoom)')
hold on
grid on
xlabel('x [m]')
ylabel('y [m]')

plotPath(SL_Plot)
if seeShape; plotRobotPath(SL_Plot); end
if ~isempty(XY_ObsGrid); plot(XY_ObsGrid(:,1),XY_ObsGrid(:,2),'r*','LineWidth',5); end
axis equal
axis([-6 6 -4 4])
end