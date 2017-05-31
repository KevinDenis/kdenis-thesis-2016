%% Init()
% initWorkspace
% load('SL_cloth.mat')
robotPose=[0 0 0];
XY_ObsGrid=[];
LSL=FreeAllPaths(LSL);
UpdatePlot(XY_ObsGrid,LSL)

%% Main()
while 1
    XY_ObsGrid=UpdateEnviroment(XY_ObsGrid);
    LSL=UpdateStateLattice(XY_ObsGrid,LSL,ObstacleTable,XY_ObsTable);
    UpdatePlot(XY_ObsGrid,LSL)
end

%% Used Functions
function StateLattice=UpdateStateLattice(XY_ObsGrid,StateLattice,ObstacleTable,XY_Table)
stringaffected=[];
xy_obs_end=XY_ObsGrid(end,:);
idxRow=findrow_mex(XY_Table,xy_obs_end);
if ~isnan(idxRow)
    IDOccPaths = [ObstacleTable(idxRow).ID];
    IdxOccPaths = [ObstacleTable(idxRow).Idx];
    for jj=1:length(IDOccPaths)
        StateLattice(IDOccPaths(jj)).free=false;
        if isempty(StateLattice(IDOccPaths(jj)).idxBlocked) || StateLattice(IDOccPaths(jj)).idxBlocked > IdxOccPaths(jj)
            StateLattice(IDOccPaths(jj)).idxBlocked=IdxOccPaths(jj);
            stringaffected=[stringaffected,num2str(IDOccPaths(jj)),'(',num2str(IdxOccPaths(jj)),') '];
        end
    end
    dispText=['obstacle ',num2str(size(XY_ObsGrid,1)), ' affects pathID(pathIdx) ', stringaffected];
    disp(dispText)
end
StateLattice=CleanupLooseStarts(StateLattice);
end

function XY_ObsGrid=UpdateEnviroment(XY_ObsGrid)
dx=0.02;
[x_obs,y_obs]=ginput2(1);
x_obs=round(round(x_obs/dx)*dx,2);
y_obs=round(round(y_obs/dx)*dx,2);
XY_ObsGrid=[XY_ObsGrid;x_obs y_obs];
end

function UpdatePlot(XY_ObsGrid,StateLattice)
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
plotRobotPath(SL_Plot)
if ~isempty(XY_ObsGrid); plot(XY_ObsGrid(:,1),XY_ObsGrid(:,2),'r*','LineWidth',5); end
axis equal
axis([-6 6 -4 4])
end