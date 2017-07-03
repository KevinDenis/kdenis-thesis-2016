function LPT_Benchmark_plot(mapSelection,ResTestGrid,saveFigure)

%% Init workspace for benchmark. 
co=get(groot,'DefaultAxesColorOrder');
stringResTestGrid={'debug';'coarse';'medium';'fine';'finer';'finest'};
load(['Benchmark',num2str(mapSelection),'_',stringResTestGrid{ResTestGrid},'.mat'])
load('ColorPalet.mat')

switch mapSelection
    case 1
        saveFileName_rawInput='EnterRobotLabEval_rawInput';
        saveFileName_rawOutput='EnterRobotLabEval_rawOutput';
        saveFileName_hist='EnterRobotLabEval_hist';
        saveFileName_map='EnterRobotLabEval_map';
        setAxis = [2 5.67 0 3.75];
    case 2
        saveFileName_rawInput='EnterRobotLabEval_narrow_rawInput';
        saveFileName_rawOutput='EnterRobotLabEval_narrow_rawOutput';
        saveFileName_hist='EnterRobotLabEval_narrow_hist';
        saveFileName_map='EnterRobotLabEval_narrow_map';
        setAxis = [2 5.67 0 3.75];
    case 3
        saveFileName_rawInput='EnterLiftEval_rawInput';
        saveFileName_rawOutput='EnterLiftEval_rawOutput';
        saveFileName_hist='EnterLiftEva_hist';
        saveFileName_map='EnterLiftEval_map';
        setAxis = [2.5 8.25 3.95 7.5]; 
    case 4
        saveFileName_rawInput='EnterLiftEval_narrow_rawInput';
        saveFileName_rawOutput='EnterLiftEval_narrow_rawOutput';
        saveFileName_hist='EnterLiftEval_narrow_hist';
        saveFileName_map='EnterLiftEval_narrow_map';
        setAxis = [2.5 8.25 3.95 7.5]; 
end

% Logic
pathInGoal_all = any([pathInGoal_circ pathInGoal_cloth],2);
pathInGoal_common=all([pathInGoal_circ pathInGoal_cloth],2);
pathInGoal_circ_U=all([pathInGoal_circ ~pathInGoal_common],2);
pathInGoal_cloth_U=all([pathInGoal_cloth ~pathInGoal_common],2);

% 3D (x y theta) --> 2D (x y) + color Data
grid_XYTH_XY=grid_XYTH(:,1:2);
[grid_XYTH_XY_S,IdxSorted]=sortrows(grid_XYTH_XY); % sorted is needed for the loop (see later)
pathInGoal_common_S=pathInGoal_common(IdxSorted,:);
pathInGoal_circ_U_S=pathInGoal_circ_U(IdxSorted,:);
pathInGoal_cloth_U_S=pathInGoal_cloth_U(IdxSorted,:);
[grid_XYTH_XY_U,idxUnique,~] = unique(grid_XYTH_XY_S,'rows','stable');
n=length(idxUnique);
pathInGoalSum_common=zeros(n,1);
pathInGoalSum_circ_U=zeros(n,1);
pathInGoalSum_cloth_U=zeros(n,1);
for ii=1:n
    if ii~=n
        idxSamePos=idxUnique(ii):idxUnique(ii+1)-1; % take all the non-unique idx intil the next unique idx.
    else
        idxSamePos=idxUnique(ii):length(IdxSorted); % don't use idxSorted(end), because this end refers to the unsorted array !
    end
    pathInGoalSum_common(ii) = sum(pathInGoal_common_S(idxSamePos));
    pathInGoalSum_circ_U(ii) = sum(pathInGoal_circ_U_S(idxSamePos));
    pathInGoalSum_cloth_U(ii) = sum(pathInGoal_cloth_U_S(idxSamePos));
end
grid_Mean = (pathInGoalSum_cloth_U-pathInGoalSum_circ_U)./(pathInGoalSum_cloth_U+pathInGoalSum_common+pathInGoalSum_circ_U);
grid_XY = grid_XYTH_XY_U;

[X_Patch,Y_Patch]=getGrid(grid_XY(:,1)',grid_XY(:,2)',grid_res);
C = discretize(grid_Mean,7);

%% Plots
figureFullScreen()
hold on
plot(Map_XY_CCW(:,1),Map_XY_CCW(:,2),'k','LineWidth',3)
plot(TestRegion(:,1),TestRegion(:,2),'r-','LineWidth',2)
plot(GoalRegion(:,1),GoalRegion(:,2),'g-','LineWidth',2)
plot(GoalTarget(:,1),GoalTarget(:,2),'g*','LineWidth',2,'MarkerSize',20)
plotRobotPose(grid_XYTH,'k')
xlabel('x [m]')
ylabel('y [m]')
hold off
axis equal
axis(setAxis)
l=legend('Obstacle','Start region','Goal region','Target','Robot start pose','Location','SE');
set(l,'FontSize',30);
set(gca,'FontSize',28)
if saveFigure; saveCurrentFigure(saveFileName_rawInput); close all; end

figureFullScreen()
hold on
plot(Map_XY_CCW(:,1),Map_XY_CCW(:,2),'k','LineWidth',3)
plot(TestRegion(:,1),TestRegion(:,2),'r-','LineWidth',2)
plot(GoalRegion(:,1),GoalRegion(:,2),'g-','LineWidth',2)
plot(GoalTarget(:,1),GoalTarget(:,2),'g*','LineWidth',2,'MarkerSize',20)
plotRobotPose(grid_XYTH(pathInGoal_common,:),'k')
plotRobotPose(grid_XYTH(pathInGoal_circ_U,:),'r')
plotRobotPose(grid_XYTH(pathInGoal_cloth_U,:),'g')
xlabel('x [m]')
ylabel('y [m]')
hold off
axis equal
axis(setAxis)
l=legend('Obstacle','Start region','Goal region','Target','Common path start pose','Circular path start pose','Clothoid path start pose','Location','SW');
set(l,'FontSize',30);
set(gca,'FontSize',28)
if saveFigure; saveCurrentFigure(saveFileName_rawOutput); close all; end

figure()
[n1, xout1] = hist(grid_Mean,7);
bar(xout1,n1,1,'FaceColor',co(1,:));
xlabel('Grid-Score')
ylabel('Occurance')
if saveFigure; saveCurrentFigure(saveFileName_hist);close all; end

fig=figureFullScreen();
fig.Renderer='Painters';
hold on
plot(Map_XY_CCW(:,1),Map_XY_CCW(:,2),'k','LineWidth',3)
plot(TestRegion(:,1),TestRegion(:,2),'r-','LineWidth',2)
plot(GoalRegion(:,1),GoalRegion(:,2),'g-','LineWidth',2)
plot(GoalTarget(:,1),GoalTarget(:,2),'g*','LineWidth',2,'MarkerSize',20)
patch(X_Patch,Y_Patch,grid_Mean)
xlabel('x [m]')
ylabel('y [m]')
colormap(ColorPalet)
colorbar
axis equal
axis(setAxis)
l=legend('Obstacle','Start region','Goal region','Target','Location','SE');
set(l,'FontSize',30);
set(gca,'FontSize',28)
if saveFigure; saveCurrentFigure(saveFileName_map); close all; end
end

%% SUB-FUNCTIONS
function [X_Patch,Y_Patch]=getGrid(X,Y,dx)
X_Patch=zeros(4,length(X));
Y_Patch=zeros(4,length(X));

X_Patch(1,:) = X-dx/2;
X_Patch(2,:) = X+dx/2;
X_Patch(3,:) = X+dx/2;
X_Patch(4,:) = X-dx/2;

Y_Patch(1,:) = Y-dx/2;
Y_Patch(2,:) = Y-dx/2;
Y_Patch(3,:) = Y+dx/2;
Y_Patch(4,:) = Y+dx/2;
end

function plotRobotPose(grid_XYTH,plotColor)
r=0.075;
X_Pose=[grid_XYTH(:,1), grid_XYTH(:,1)+r*cos(grid_XYTH(:,3)), nan(size(grid_XYTH(:,1)))].';
Y_Pose=[grid_XYTH(:,2), grid_XYTH(:,2)+r*sin(grid_XYTH(:,3)), nan(size(grid_XYTH(:,1)))].';
plot(X_Pose(:).',Y_Pose(:).',plotColor,'LineWidth',1.5)
end