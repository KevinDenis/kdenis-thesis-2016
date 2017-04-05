clearvars -except StateLattice MotionPrem
% if ~exist('StateLattice', 'var')
%     load('StateLattice.mat');
% end
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                           %
%       State Latice Matrix Structure       %
%  [ x0 y0 th0 x1 y1 th1 k dk L intKappa ]  %
%    1  2   3  4  5   6  7  8 9    10       %
%                                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% TODO
% * Not all start positions are end positions

addpath('ClothoidG1fitting');
load('SimpleRobotPlot.mat')

LabBW = ~imread('ComplicatedLab.bmp');
LabGrid = robotics.BinaryOccupancyGrid(LabBW,100);
LabGrid.GridLocationInWorld=[-5 -5];
% 


%% Selecting Free Paths
selectFree=[StateLattice.free].';
SL=StateLattice(selectFree);

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

xyth_start=[-2 1 pi/2];
xyth_end=[4 4 0];

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
%% plot optimal path

% xy_all = unique(xyth_all(:,1:2), 'rows');


if true
    figure(1)
    show(LabGrid)
    hold on
%     scatter(xy_all(:,1),xy_all(:,2),[],'k*')
    for ii=1:length(optimal_edges)
        kk=optimal_edges(ii);
        plot(SL(kk).X,SL(kk).Y,'g','LineWidth',3)
        scatter(SL(kk).x0,SL(kk).y0,[],'r*')
        scatter(SL(kk).x1,SL(kk).y1,[],'r*')
        plotRobotPath(SimpleRobotPlot,SL(kk).X(1:5:end),SL(kk).Y(1:5:end),SL(kk).TH(1:5:end))
    end
    
    grid minor
    hold off
%     l1=legend('shifted motion primitive','optimal path','visited nodes','Location','NW');
%     set(l1,'FontSize',12);
end
%}