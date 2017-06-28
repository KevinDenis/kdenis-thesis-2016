initWorkspace
co=get(groot,'DefaultAxesColorOrder');

%% Init workspace for benchmark. 
% Additional Data
ResTestGrid = 1;
stringResTestGrid={'coarse';'normal';'fine'};
load(['Benchmark1_',stringResTestGrid{ResTestGrid},'.mat'])
load('RobotLaboEntrance_Corners_XY_CCW.mat')
Map_XY_CCW=RobotLaboEntrance_Corners_XY_CCW;
map = 'RobotLaboEntranceEdges.bmp';

RobotHull=[-0.80  0.20;
           -0.46  0.30;
            0.50  0.30;
            0.50 -0.30;
           -0.46 -0.30;
           -0.80 -0.20;
           -0.80  0.20];
TestRegion = [3.0, 0.5;
              3.0, 3.5;
              5.0, 3.5;
              5.0, 2.8;
              5.5, 2.8;
              5.5, 1.4;
              5.0, 1.4;
              5.0, 0.5;
              3.0, 0.5];
GoalRegion = [2.40, 2.20;
              2.56, 2.20;
              2.56, 2.94;
              2.40, 2.94;
              2.40, 2.20];

% Color Map
Color7=rgb('DeepSkyBlue');
Color6=rgb('RoyalBlue');
Color5=rgb('MidnightBlue');
Color4=rgb('Black');
Color3=rgb('DarkRed');
Color2=rgb('FireBrick');
Color1=co(2,:);

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
figure()
plotRobotPose(grid_XYTH,'k')

figure()
hold on
plot(Map_XY_CCW(:,1),Map_XY_CCW(:,2),'k','LineWidth',3)
plot(TestRegion(:,1),TestRegion(:,2),'r-')
plot(GoalRegion(:,1),GoalRegion(:,2),'g-')
plotRobotPose(grid_XYTH(pathInGoal_common,:),'k')
plotRobotPose(grid_XYTH(pathInGoal_circ_U,:),'r')
plotRobotPose(grid_XYTH(pathInGoal_cloth_U,:),'g')
hold off

figure()
[n1, xout1] = hist(grid_Mean,7);
bar(xout1,n1,1,'FaceColor',co(1,:));
xlabel('Grid-Score')
ylabel('Occurance')

figure()
hold on
plot(Map_XY_CCW(:,1),Map_XY_CCW(:,2),'k','LineWidth',3)
plot(TestRegion(:,1),TestRegion(:,2),'r-')
plot(GoalRegion(:,1),GoalRegion(:,2),'g-')
patch(X_Patch,Y_Patch,grid_Mean)
colormap([Color1;Color2;Color3;Color4;Color5;Color6;Color7])
colorbar
% axis([3 6 0 4])
axis normal

%% FUNCTIONS
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
r=0.04;
X_Pose=[grid_XYTH(:,1), grid_XYTH(:,1)+r*cos(grid_XYTH(:,3)), nan(size(grid_XYTH(:,1)))].';
Y_Pose=[grid_XYTH(:,2), grid_XYTH(:,2)+r*sin(grid_XYTH(:,3)), nan(size(grid_XYTH(:,1)))].';
plot(X_Pose(:).',Y_Pose(:).',plotColor,'LineWidth',1.5)
end