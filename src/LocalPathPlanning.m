%=========================================================================%
%                                                                         %
%                            Local Path Planning                          %
%                            -------------------                          %

%                                                                         %
% Overview :                                                              %
%   * This function will mimic the plan recognition algorithm.            %
%   The user selects a path by clicking on it. The closest path to this   %
%   input will be chosen. If this path is not directly connected to the   %
%   robot pose, the path connecting the robot pose and the destination is %
%   searched. Since one path is composed at most by 2 subsequent curves,  %
%   this process is only done once                                        %
%   * In a previous version of this code, Dijkstra’s algorithm was used to %
%   do this. This was a bit "overkill" since in this case "a" solution is %
%   also the "optimal" solution. Only one path leads to a certain         %
%   intermediate destination (the expansion position), where the          %
%   second curve originates.                                              %
%   This has been commented out at the end of this function               %
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

function LocalPathPlanning(LSL_W,LabGrid,robotPose)
close all
figureFullScreen();
show(LabGrid)
title('Select desired path by clicking on it')
hold on
grid on
xlabel('x [m]')
ylabel('y [m]')
axis equal
plotPath(LSL_W)
plotRoboticWheelchair(robotPose)
set(gca,'FontSize',14)
set(gca, 'box', 'off')

[x_path,y_path]=ginput(1);
goalIdx=getIdxClossestPath(LSL_W,x_path,y_path);

vertices = [[LSL_W.x0].', [LSL_W.y0].',[LSL_W.th0].',[LSL_W.x1].',[LSL_W.y1].',[LSL_W.th1].'];
if ~IsNear(vertices(goalIdx,1:3),robotPose,1e-3)
    idxStart=findrow_mex(vertices(:,4:6),vertices(goalIdx,1:3));
    goalIdx=[idxStart;goalIdx];
end
%% plot optimal path

    figureFullScreen();
    show(LabGrid)
    title('Local Path Planning based on collision-free trajectories')
    hold on
    plotRobotPath(LSL_W(goalIdx))
    hold off
    set(gca,'FontSize',14)
    set(gca, 'box', 'off')
%     pause()
%     saveCurrentFigure
end

function goalIdx=getIdxClossestPath(LSL_W,x_path,y_path)   
currentMinDist=inf;
for ii=1:length(LSL_W)
    if ~LSL_W(ii).free
        idxBlocked=LSL_W(ii).idxBlocked;
        dX_ii=LSL_W(ii).X(1:idxBlocked-1)- x_path;
        dY_ii=LSL_W(ii).Y(1:idxBlocked-1)- y_path;
    else
        dX_ii = [LSL_W(ii).X] - x_path;
        dY_ii = [LSL_W(ii).Y] - y_path;
    end
    distToPath = DNorm2([dX_ii dY_ii],2);
    minDist=min(distToPath);
    if minDist < currentMinDist
        currentMinDist=minDist;
        goalIdx = ii;
    end
end
end

%% OLD METHOD WITH DIJCKSTA'S ALGORITM, PLEASE DISCARD
%{
%% Building vertices
vx=[[LSL_W.x0].' [LSL_W.x1].'];
vy=[[LSL_W.y0].' [LSL_W.y1].'];
vth=[[LSL_W.th0].' [LSL_W.th1].'];

%% construct graph
xyth_all = unique([vx(:) vy(:) vth(:)], 'rows');
xyth_0 = [vx(:,1) vy(:,1) vth(:,1)];
xyth_1 = [vx(:,2) vy(:,2) vth(:,2)];

ii_temp =  findVector(xyth_all,xyth_0);
jj_temp =  findVector(xyth_all,xyth_1);
ii=[ii_temp;jj_temp];
jj=[jj_temp;ii_temp];

pathCost=[LSL_W.pathCost].';

tic
G = sparse(ii,jj,[pathCost;pathCost],length(xyth_all),length(xyth_all));
toc
% figure()
% spy(G)


%% starting index and destination index
tic

xyth_start=robotPose;
xyth_end=[LSL_W(goalIdx).x1, LSL_W(goalIdx).y1, LSL_W(goalIdx).th1];

[~,st_idx] = min(DNorm2(xyth_all-xyth_start,2));
[~,dest_idx] = min(DNorm2(xyth_all-xyth_end,2));
% dest_idx =findrow_mex(round(xyth_all,3),xyth_end);
All_Edges = [vx(:,1) vy(:,1) vth(:,1) vx(:,2) vy(:,2) vth(:,2)];

%% dijkstra's algorithm to find shortes path
[~,path,~]=graphshortestpath(G,st_idx,dest_idx);
xyth_opt_path = xyth_all(path,:);
optimal_edges = zeros(size(xyth_opt_path,1)-1,1);
for ii=1:size(xyth_opt_path,1)-1
    v_sd=[xyth_opt_path(ii,:) xyth_opt_path(ii+1,:)];
    optimal_edges(ii)=findVector(All_Edges,v_sd);
end
toc
OptPath=LSL_W(optimal_edges);
%}
