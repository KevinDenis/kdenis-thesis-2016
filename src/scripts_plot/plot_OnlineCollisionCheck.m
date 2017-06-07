%% Init()
initPlotScripts
load('LSL_cloth.mat')
load('ObstacleTable_cloth.mat')
load('XY_ObsTable_cloth.mat')


robotPose=[0 0 0];
XY_ObsGrid=[];
LSL=FreeAllPaths(LSL);
% UpdatePlot(XY_ObsGrid,StateLattice)

XY_ObsGrid=[-1.60 0.2;
             3.90 2.96;
             1.94 -1.82;
             4,0;];

%% Main()

affected_paths=[];
for ii = 1:size(XY_ObsGrid,1)
    XY_ObsGrid_ii=XY_ObsGrid(1:ii,:);
    [LSL,affected_paths_ii]=UpdateStateLattice(XY_ObsGrid_ii,LSL,ObstacleTable,XY_ObsTable);
    affected_paths=[affected_paths;affected_paths_ii];
end

UpdatePlot(XY_ObsGrid_ii,LSL,affected_paths)

saveCurrentFigure('LPT_LT');

%% Used Functions
function [StateLattice,affected_paths]=UpdateStateLattice(XY_ObsGrid,StateLattice,ObstacleTable,XY_Table)
affected_paths=[];
stringaffected=[];
xy_obs_end=XY_ObsGrid(end,:);
% tic
idxRow=findrow_mex(XY_Table,xy_obs_end);
if ~isnan(idxRow)
    IDOccPaths = [ObstacleTable(idxRow).ID];
    IdxOccPaths = [ObstacleTable(idxRow).Idx];
    for jj=1:length(IDOccPaths)
        StateLattice(IDOccPaths(jj)).free=false;
        if isempty(StateLattice(IDOccPaths(jj)).idxBlocked) || StateLattice(IDOccPaths(jj)).idxBlocked > IdxOccPaths(jj)
            StateLattice(IDOccPaths(jj)).idxBlocked=IdxOccPaths(jj);
%             stringaffected=[stringaffected,num2str(IDOccPaths(jj)),'(',num2str(IdxOccPaths(jj)),') '];
%             affected_paths = [affected_paths;IDOccPaths(jj)];
        end
    end
%     dispText=['obstacle ',num2str(size(XY_ObsGrid,1)), ' affects pathID(pathIdx) ', stringaffected];
%     disp(dispText)
end
% toc
StateLattice=CleanupLooseStarts(StateLattice);
end

function UpdatePlot(XY_ObsGrid,StateLattice,affected_paths)
co=get(groot,'DefaultAxesColorOrder');

fig=figureFullScreen();
fig.Renderer='Painters';
title('')
hold on
grid on
xlabel('x [m]')
ylabel('y [m]')



text(0.5,3.5,['obstacle affects : 763(202) 764(192) 779(226)' newline 'pathID(pathIdx)'],'FontSize',20)

p1 = [2 3.5];                         % First Point
p2 = XY_ObsGrid(2,:);                         % Second Point
dp = p2-p1;                         % Difference

% plotPath(FreeAllPaths(StateLattice(affected_paths)),co(2,:),1,'--');
plotPath(StateLattice)


% vertices=[[StateLattice.x1].',[StateLattice.y1].',[StateLattice.th1].'];
% idxToSee=find(ismember(vertices,[2.5,-2.5 -pi*3/8],'rows'))
% plotRobotPath(StateLattice([479 466]));
%  306

  % 462
% plotRobotPath(StateLattice([306 466]));

plotRoboticWheelchair([0 0 0])
if ~isempty(XY_ObsGrid); plot(XY_ObsGrid(:,1),XY_ObsGrid(:,2),'o','Color',co(2,:),'MarkerFaceColor',co(2,:),'LineWidth',7); end
% 'Adjusted path length'
l=legend('Collision-free paths','Wheelchair footprint','Obstacles','Location','SW');
set(l,'FontSize',26);
set(gca,'FontSize',24)
axis equal
axis([-4.5 4.5 -3.5 4])
% annotation('arrow',[0.65 0.743],[0.86,0.815])
annotation('arrow',[0.68 0.785],[0.91,0.865])
% quiver(p1(1),p1(2),dp(1),dp(2),0,'Color',[0 0 0],'LineWidth',1.5)
end