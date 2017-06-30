function LPT_Benchmark_calc(mapSelection,ResTestGrid)
%#ok<*PFOUS,*PFBNS> % MATLAB, don't show PARFOR-warning 
%% Init workspace for benchmark
stringResTestGrid={'coarse';'medium';'fine';'finest'};
saveFileName=['data_mat/Benchmark',num2str(mapSelection),'_',stringResTestGrid{ResTestGrid},'.mat'];
fprintf('loading data for benchmark %d using %s grid ...',mapSelection,stringResTestGrid{ResTestGrid});
tic
% Will load data unless it is allready in workspace (old implementation
% when this file was a script ...)
if ~exist('LSLset', 'var'); load('LSLset.mat'); end
if ~exist('grid_XY', 'var'); load('grid_XY.mat'); end
if ~exist('LSL_cloth', 'var'); load('LSL_cloth.mat'); LSL_cloth = LSL; end
if ~exist('ObstacleTable_cloth', 'var'); load('ObstacleTable_cloth.mat'); ObstacleTable_cloth=ObstacleTable; end
if ~exist('XY_ObsTable_cloth', 'var'); load('XY_ObsTable_cloth.mat'); XY_ObsTable_cloth=XY_ObsTable; end
if ~exist('LSL_circ', 'var'); load('LSL_circ.mat'); LSL_circ = LSL; end
if ~exist('ObstacleTable_circ', 'var'); load('ObstacleTable_circ.mat'); ObstacleTable_circ=ObstacleTable; end
if ~exist('XY_ObsTable_circ', 'var'); load('XY_ObsTable_circ.mat'); XY_ObsTable_circ=XY_ObsTable; end

switch ResTestGrid
    case 1 % coarse Test Grid
        grid_res = 0.1;
        angle_res = pi/4;
    case 2 % normal Test Grid
        grid_res = 0.05;
        angle_res = pi/8;
    case 3 % fine Test Grid
        grid_res = 0.025;
        angle_res = pi/16;
    case 4 % finest Test Grid
        grid_res = 0.025;
        angle_res = pi/32;
end

switch mapSelection
    case 1 % Robot Labo Entrance Benchmark with 80cm door (20cm play)
        angle_range = pi/2;
        TH = wrap2Pi(-angle_range:angle_res:angle_range);
        driveDir = 'fwd';
        map = 'RobotLaboEntranceEdges.bmp';
        load('RobotLaboEntrance_Corners_XY_CCW.mat')
        Map_XY_CCW=RobotLaboEntrance_Corners_XY_CCW;
        SimpleMapXY = Map_XY_CCW;
        SimpleMapXY(9,1)=8;
        SimpleMapXY(10,:)=[];
        SimpleMapXY(10,1)=8;
        SimpleMapXY=[SimpleMapXY;[0.01,0.01;2.11,0.01]];
        TestRegion =   [3.0, 0.5;
                        3.0, 3.5;
                        5.0, 3.5;
                        5.0, 2.8;
                        5.5, 2.8;
                        5.5, 1.4;
                        5.0, 1.4;
                        5.0, 0.5;
                        3.0, 0.5];

        GoalRegion =   [2.40, 2.20;
                        2.56, 2.20;
                        2.56, 2.94;
                        2.40, 2.94;
                        2.40, 2.20];
        GoalTarget = mean(GoalRegion(1:end-1,:),1);
    case 2 % Robot Labo Entrance Benchmark with 70cm door (10cm play)
        angle_range = pi/2;
        TH = wrap2Pi(-angle_range:angle_res:angle_range);
        driveDir = 'fwd';
        map = 'RobotLaboEntranceEdges_narrow.bmp';
        load('RobotLaboEntrance_Corners_XY_CCW_narrow.mat')
        Map_XY_CCW=RobotLaboEntrance_Corners_XY_CCW_narrow;
        SimpleMapXY = Map_XY_CCW;
        SimpleMapXY(9,1)=8;
        SimpleMapXY(10,:)=[];
        SimpleMapXY(10,1)=8;
        SimpleMapXY=[SimpleMapXY;[0.01,0.01;2.11,0.01]];
        TestRegion =   [3.0, 0.5;
                        3.0, 3.5;
                        5.0, 3.5;
                        5.0, 2.8;
                        5.5, 2.8;
                        5.5, 1.4;
                        5.0, 1.4;
                        5.0, 0.5;
                        3.0, 0.5];

        GoalRegion =   [2.40, 2.20;
                        2.56, 2.20;
                        2.56, 2.84;
                        2.40, 2.84;
                        2.40, 2.20];
        GoalTarget = mean(GoalRegion(1:end-1,:),1);
    case 3 % Lift Entrance Benchmark
        angle_range = pi/2;
        TH = wrap2Pi((-angle_range:angle_res:angle_range)+pi);
        driveDir = 'bkw';
        map = 'LiftEdges.bmp';
        load('LiftEntrance_Corners_XY_CCW.mat')
        Map_XY_CCW=LiftEntrance_Corners_XY_CCW;

        SimpleMapXY = [Map_XY_CCW;Map_XY_CCW(1,:)];

        TestRegion = [3.0, 5.5;
                      6.7, 5.5;
                      6.7, 7.3;
                      3.0, 7.3;
                      3.0, 5.5];

        GoalRegion = [6.8, 4.4;
                      7.2, 4.4;
                      7.2, 5.2;
                      6.8, 5.2;
                      6.8, 4.4];
        GoalRegion=flip(GoalRegion);
        GoalTarget = mean(GoalRegion(1:end-1,:),1);
        GoalTarget(1) = 6;
    case 4 % Lift Entrance Benchmark
        angle_range = pi/2;
        TH = wrap2Pi((-angle_range:angle_res:angle_range)+pi);
        driveDir = 'bkw';
        map = 'LiftEdges_narrow.bmp';
        load('LiftEntrance_Corners_XY_CCW_narrow.mat')
        Map_XY_CCW=LiftEntrance_Corners_XY_CCW_narrow;

        SimpleMapXY = [Map_XY_CCW;Map_XY_CCW(1,:)];

        TestRegion = [3.0, 5.5;
                      6.7, 5.5;
                      6.7, 7.3;
                      3.0, 7.3;
                      3.0, 5.5];

        GoalRegion = [6.8, 4.4;
                      7.2, 4.4;
                      7.2, 5.2;
                      6.8, 5.2;
                      6.8, 4.4];
        GoalRegion=flip(GoalRegion);
        GoalTarget = mean(GoalRegion(1:end-1,:),1);
        GoalTarget(1) = 6;
end

[grid_XY]=polygrid(TestRegion(:,1),TestRegion(:,2),(1/grid_res)^2);

grid_XYTH_tmp=addTHtoGridXY(grid_XY,TH);
grid_XYTH = grid_XYTH_tmp;
idxVec=1:length(TH);
lenVec=length(idxVec);
for ii=1:size(grid_XY,1)
    kk=(ii-1)*lenVec+idxVec;
    dP = GoalTarget-grid_XY(ii,:);
    angle_shift = atan2(dP(2),dP(1));
    TH_new = wrap2Pi(round((TH+angle_shift)/angle_res)*angle_res);
    grid_XYTH(kk,3)=TH_new;
end

RobotHull= [-0.80,  0.20;
            -0.46,  0.30;
             0.50,  0.30;
             0.50, -0.30;
            -0.46, -0.30;
            -0.80, -0.20;
            -0.80,  0.20];

RobotHull=LinInterpToRes(RobotHull,0.05);

fprintf(' done ! (took %2.3f sec) \n',toc)
fprintf('Calculating feasable poses in test region...')
tic
toKeep=false(length(grid_XYTH),1);
for ii = 1:length(grid_XYTH)
    robotPose = grid_XYTH(ii,:);
    RobotHull_RotTrans = RotTransXY(RobotHull,robotPose(3),robotPose(1),robotPose(2));
    if all(InPolygon(RobotHull_RotTrans(:,1),RobotHull_RotTrans(:,2),SimpleMapXY(:,1),SimpleMapXY(:,2)))
        toKeep(ii)=1;
    end
end
fprintf(' done ! (took %3.0f msec) \n',toc*1e3)
grid_XYTH=grid_XYTH(toKeep,:);
figure()
hold on
plot(Map_XY_CCW(:,1),Map_XY_CCW(:,2),'k','LineWidth',3)
plot(TestRegion(:,1),TestRegion(:,2),'r-')
plot(GoalRegion(:,1),GoalRegion(:,2),'g-')
plot(GoalTarget(1),GoalTarget(2),'*g','LineWidth',3)
plotRobotPose(grid_XYTH,'k')
drawnow % user can imidiatly see if he has made a mistake in the input parameters

pathInGoal_cloth=false(length(grid_XYTH),1);
pathInGoal_circ=false(length(grid_XYTH),1);
n=length(grid_XYTH);
fprintf('Calculating feasable paths in  goal region...')
tic
ppm = ParforProgressStarter2('Calculating...', n, 0.1, 0, 1, 1);
parfor ii=1:n
    [LSL_W_cloth,~,~]=BuildLSLColFree(LSL_cloth,ObstacleTable_cloth,XY_ObsTable_cloth,grid_XY,map,grid_XYTH(ii,:),driveDir,0);
    [LSL_W_circ,~,~]=BuildLSLColFree(LSL_circ,ObstacleTable_circ,XY_ObsTable_circ,grid_XY,map,grid_XYTH(ii,:),driveDir,0);
    pathInGoal_cloth(ii)=pathThroughGoalRegion(GoalRegion,LSL_W_cloth);
    pathInGoal_circ(ii)=pathThroughGoalRegion(GoalRegion,LSL_W_circ); 
    ppm.increment(ii);
%         if pathInGoal_cloth(ii)
%             clf
%             figure(1)
%             hold on
%             plot(Map_XY_CCW(:,1),Map_XY_CCW(:,2),'k','LineWidth',3)
%             plot(TestRegion(:,1),TestRegion(:,2),'r-')
%             plot(GoalRegion(:,1),GoalRegion(:,2),'g-')
%             plotRobotPose(grid_XYTH,'k')
%             plotRobotPath(LSL_W_cloth)
%             hold off
%             drawnow
% %             disp(pathInGoal_cloth(ii))
% %             disp(ii)
%         end
end
delete(ppm);
fprintf(' done ! (took %3.3f min) \n',toc/60)
fprintf('saving data for benchmark %d using %s grid ...',mapSelection,stringResTestGrid{ResTestGrid});
tic
save(saveFileName,'angle_res','grid_res','GoalRegion','grid_XY','grid_XYTH','GoalTarget','pathInGoal_circ','pathInGoal_cloth','TestRegion','TH','Map_XY_CCW','SimpleMapXY')
fprintf(' done ! (took %2.3f msec) \n',toc*1e3)
end
%% SUB-FUNCTIONS
function pathInGoal=pathThroughGoalRegion(GoalRegion,Path)
for ii=1:length(Path)
    if ~Path(ii).free
        idxPlot=Path(ii).idxBlocked;
        Path(ii).X=Path(ii).X(1:idxPlot-1);
        Path(ii).Y=Path(ii).Y(1:idxPlot-1);
        Path(ii).TH=Path(ii).TH(1:idxPlot-1);
    end
end

ii_todelete=zeros(1);
nn=1;
for ii=1:length(Path)
    if isempty(Path(ii).X)
        ii_todelete(nn)=ii;
        nn=nn+1;
    end
end
if nn ~= 1
    Path(ii_todelete)=[];
end

X_path=zeros(1);
Y_path=zeros(1);
kk=0;
for ii=1:length(Path)
    kk=kk(end)+(1:(length(Path(ii).X)+1));
    X_path(kk)=[Path(ii).X;nan];
    Y_path(kk)=[Path(ii).Y;nan];
end

if any(InPolygon(X_path,Y_path,GoalRegion(:,1),GoalRegion(:,2)))
    pathInGoal=true;
else
    pathInGoal = false;
end
end

function plotRobotPose(grid_XYTH,plotColor)
r=0.025;
X_Pose=[grid_XYTH(:,1), grid_XYTH(:,1)+r*cos(grid_XYTH(:,3)), nan(size(grid_XYTH(:,1)))].';
Y_Pose=[grid_XYTH(:,2), grid_XYTH(:,2)+r*sin(grid_XYTH(:,3)), nan(size(grid_XYTH(:,1)))].';
plot(X_Pose(:).',Y_Pose(:).',plotColor,'LineWidth',1.5)
end

function grid_X_Y_TH=addTHtoGridXY(grid_X_Y,TH)
%grid_X_Y_TH=addTHtoGridXY(grid_X_Y,TH)
%   Adds end positions pased on TH to grid_X_Y
grid_X_Y_rep=repmat(grid_X_Y,length(TH),1);
TH_rep=repelem(TH,length(grid_X_Y)).';
grid_X_Y_TH=[grid_X_Y_rep TH_rep];
grid_X_Y_TH=unique(grid_X_Y_TH,'rows');
end