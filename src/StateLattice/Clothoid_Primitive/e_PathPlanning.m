clearvars -except grid_XY LSLset StateLattice SL_W robotPose LabGrid
% if ~exist('StateLattice', 'var')
%     load('StateLattice.mat');
% end
close all
clc



% LabBW = ~imread('ComplicatedLab.bmp');
% LabGrid_R = robotics.BinaryOccupancyGrid(LabBW,100);
% LabGrid_R.GridLocationInWorld=[-5 -5];
% 

showPlot=true;

%% Selecting Free Paths
SL=SL_W;

%% Building vertices
vx=[[SL.x0].' [SL.x1].'];
vy=[[SL.y0].' [SL.y1].'];
vth=[[SL.th0].' [SL.th1].'];

%% construct graph
xyth_all = unique([vx(:) vy(:) vth(:)], 'rows');
xyth_0 = [vx(:,1) vy(:,1) vth(:,1)];
xyth_1 = [vx(:,2) vy(:,2) vth(:,2)];

ii_temp =  findVector(xyth_all,xyth_0);
jj_temp =  findVector(xyth_all,xyth_1);
ii=[ii_temp;jj_temp];
jj=[jj_temp;ii_temp];

% dispCost=1*StateLaticeMatrix_Augmented(:,end)+3*StateLaticeMatrix_Augmented(:,end-1);

pathCost=[SL.pathCost].';
% dispCost
% G = sparse(length(xyth_all),length(xyth_all));
tic
G = sparse(ii,jj,[pathCost;pathCost],length(xyth_all),length(xyth_all));
toc
% figure()
% spy(G)


%% starting index and destination index
tic

xyth_start=robotPose;
xyth_end=[2.24 3.91 0];

st_idx = findVector(xyth_all,xyth_start);
dest_idx = findVector(xyth_all,xyth_end);
All_Edges = [vx(:,1) vy(:,1) vth(:,1) vx(:,2) vy(:,2) vth(:,2)];

%% dijestra's algoritm to find shortes path
[dist,path,pred]=graphshortestpath(G,st_idx,dest_idx);
xyth_opt_path = xyth_all(path,:);
optimal_edges = zeros(size(xyth_opt_path,1)-1,1);
for ii=1:size(xyth_opt_path,1)-1
    v_sd=[xyth_opt_path(ii,:) xyth_opt_path(ii+1,:)];
    optimal_edges(ii)=findVectorRev(All_Edges,v_sd);
end
toc
OptPath=SL(optimal_edges);

%% plot optimal path

if showPlot
    figure(1)
    show(LabGrid)
    title('Local Path Planning')
    hold on
    plotRobotPath(OptPath)
    hold off
    l1=legend('optimal path','visited nodes','robot');
    set(l1,'FontSize',12);
%     pause()
%     saveCurrentFigure
end
%}