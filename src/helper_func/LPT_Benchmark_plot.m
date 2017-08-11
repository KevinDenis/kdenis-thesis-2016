function LPT_Benchmark_plot(mapSelection,ResTestGrid,saveFigure)

%% Init workspace for benchmark. 
co=get(groot,'DefaultAxesColorOrder');
stringResTestGrid={'debug';'coarse';'medium';'fine';'finer';'finest'};
load(['Benchmark',num2str(mapSelection),'_',stringResTestGrid{ResTestGrid},'.mat'])
load('ColorPalet.mat')

switch mapSelection
    case 1
        saveFileName_input='EnterRobotLabEval_input';
        saveFileName_result='EnterRobotLabEval_result';
        saveFileName_hist='EnterRobotLabEval_hist';
        setAxis = [2 5.67 0 3.75];
    case 2
        saveFileName_input='EnterRobotLabEval_narrow_input';
        saveFileName_result='EnterRobotLabEval_narrow_result';
        saveFileName_hist='EnterRobotLabEval_narrow_hist';
        setAxis = [2 5.67 0 3.75];
    case 3
        saveFileName_input='EnterLiftEval_input';
        saveFileName_result='EnterLiftEval_result';
        saveFileName_hist='EnterLiftEva_hist';
        setAxis = [2.5 8.25 3.95 7.5]; 
    case 4
        saveFileName_input='EnterLiftEval_narrow_input';
        saveFileName_result='EnterLiftEval_narrow_result';
        saveFileName_hist='EnterLiftEval_narrow_hist';
        setAxis = [2.5 8.25 3.95 7.5]; 
end

% Logic
pathInGoal_all = any([pathInGoal_circ pathInGoal_cloth],2);
pathInGoal_common=all([pathInGoal_circ pathInGoal_cloth],2);
pathInGoal_circ_U=all([pathInGoal_circ ~pathInGoal_common],2);
pathInGoal_cloth_U=all([pathInGoal_cloth ~pathInGoal_common],2);

startPoseScore = [-ones(size(find(pathInGoal_circ_U)));zeros(size(find(pathInGoal_common)));ones(size(find(pathInGoal_cloth_U)))];

%% Command window info
fprintf('\n')
fprintf('-------------------------------------------------\n')
fprintf('Map selection : %d',mapSelection)
fprintf('\n')
fprintf('path in goal all %d\n',sum(pathInGoal_all))
fprintf('path in goal circulat LPT %d\n',sum(pathInGoal_circ_U))
fprintf('path in goal both LPTs %d\n',sum(pathInGoal_common))
fprintf('path in goal clothoidal LPT %d\n',sum(pathInGoal_cloth_U))


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
if saveFigure; saveCurrentFigure(saveFileName_input); close all; end

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
if saveFigure; saveCurrentFigure(saveFileName_result); close all; end

figure()
offSetTextHor = -0.13;
offSetTextVert = 100;
[n1,  ~] = hist(startPoseScore,3);
n1_percentage = n1./sum(n1)*100;
bar([-1,0,1],n1,0.5,'FaceColor',co(1,:));
UniqueCircPerc_str = [num2str(n1_percentage(1),2),'%'];
CommonPerc_str = [num2str(n1_percentage(2),2),'%'];
UniqueClothPerc_str = [num2str(n1_percentage(3),2),'%'];
grid on
grid minor
axis([-1.5 1.5 0 2000])
ylabel('Successful start poses')
set(gca,'XTickLabel',{'Only Circular LPT' 'Both LPTs' 'Only Clothoid LPT'},'FontSize',13)
text(-1+offSetTextHor,n1(1)+offSetTextVert,UniqueCircPerc_str,'FontSize',15)
text( 0+offSetTextHor ,n1(2)+offSetTextVert,CommonPerc_str,'FontSize',15)
text( 1+offSetTextHor,n1(3)+offSetTextVert,UniqueClothPerc_str,'FontSize',15)
if saveFigure; saveCurrentFigure(saveFileName_hist);close all; end
end

%% SUB-FUNCTIONS
function plotRobotPose(grid_XYTH,plotColor)
r=0.075;
X_Pose=[grid_XYTH(:,1), grid_XYTH(:,1)+r*cos(grid_XYTH(:,3)), nan(size(grid_XYTH(:,1)))].';
Y_Pose=[grid_XYTH(:,2), grid_XYTH(:,2)+r*sin(grid_XYTH(:,3)), nan(size(grid_XYTH(:,1)))].';
plot(X_Pose(:).',Y_Pose(:).',plotColor,'LineWidth',1.5)
end