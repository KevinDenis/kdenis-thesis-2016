%=========================================================================%
%                                                                         %
%              Application of LPT on Discrete Motion Planning             %
%              ----------------------------------------------             %
%                          Part 3. Path Planning                          %
%                                                                         %
% Overview :                                                              %
%   * Plan a path from the centre of the robot lab to the elevator        %
% The wheelchair has to enter the elevator "backwards", but should plan to%
% Drive the least amount of time in reverse, this is not comfortable for  %
% the use.                                                                %
%   * First, a 2D planner is used, based on the Voronoi diagram of the map%
% This maximises the distance to each obstacle, therefore the map is not  %
% inflated before the use of this 2D planner.                             %
%   * Then, at certain distances of on that path (maxNodeDist), a State   %
% Lattice is drawn, indicating feasible connections in the surrounding of %
% that position.                                                          %
%   * Obstacle free paths (this time, not length adjusted, just removed)  %
% are calculated and then Dijkstra's algorithm is used to find a feasible %
% path from start to end pose.                                            %
%                                                                         %
% Kevin DENIS, KU Leuven, 2016-17                                         %
% Master Thesis: Path planning algorithm for semi-autonomous              %
%                mobile robots with fast and accurate collision checking  %
%                                                                         %
%=========================================================================%

initWorkspace

%% 2D planner
pathRes = 0.25;
maxNodeDist = 0.5;
% load('LSL_DMP.mat')
xy_start=[5.5 8];
xy_end=[19.75 16.25];
xyth_start = [xy_start pi/2];
xyth_end = [xy_end pi];
voronoi_path= VoronoiOptimalPath(xy_start, xy_end,pathRes,maxNodeDist);
[map,LabGridEdges,XY_occ_lab] = getMapEdgeXYOccFormBmp('RobotLabo_Lift_Black.bmp',0.02);
LabGrid = map;

%% State Lattice Creation on 2D paths & calculating free paths
x_sift_vec=voronoi_path(:,1);
y_sift_vec=voronoi_path(:,2);
growthFactor=length(x_sift_vec);
lengthLSL=length(LSL);
lengthGSL=growthFactor*lengthLSL;
GSL(1,1)=LSL(1);
GSL(lengthGSL,1)=LSL(end);
GSL=GSL.';
kk=1:lengthLSL;
for ii=1:growthFactor
	GSL(kk)=LSL;
	kk=kk+lengthLSL;
end

tic
fprintf('Building Global State Lattice ...')
counter=0;
kk=1:length(LSL);
for ll=1:length(x_sift_vec)
    x_shift=x_sift_vec(ll);
    y_shift=y_sift_vec(ll);
    for ii=kk
        counter=counter+1;
        PathOccXY_ii        = GSL(ii).PathOccXY + repmat([x_shift y_shift],1);
        GSL(ii).x0          = GSL(ii).x0+x_shift;
        GSL(ii).y0          = GSL(ii).y0+y_shift;
        GSL(ii).x1          = GSL(ii).x1+x_shift;
        GSL(ii).y1          = GSL(ii).y1+y_shift;
        GSL(ii).X           = GSL(ii).X+x_shift;
        GSL(ii).Y           = GSL(ii).Y+y_shift;
        GSL(ii).PathOccXY   = PathOccXY_ii;
        GSL(ii).ID          = counter;
    end
    kk=kk+length(LSL);
end
fprintf(' done ! (took %2.3f sec) \n',toc)

tic
progressbar('Calculating Collision Free Paths')
fprintf('Calculating Collision Free Paths ...')
n=length(GSL);
for ii=1:n
    PathOccXY_ii=GSL(ii).PathOccXY;
%     PathOccXY_ii=PathOccXY_ii(:,1:2);
    % check whether path is outside world limits, blocked if true
    if any(PathOccXY_ii(:,1)<(map.XWorldLimits(1))+0.5) || ...
       any(PathOccXY_ii(:,1)>(map.XWorldLimits(2))-0.5) || ...
       any(PathOccXY_ii(:,2)<(map.YWorldLimits(1))+0.5) || ...
       any(PathOccXY_ii(:,2)>(map.YWorldLimits(2))-0.5)
       GSL(ii).free=false;
    else
        if any(getOccupancy(map,PathOccXY_ii))
            GSL(ii).free=false; % actual collision checking. No lookup table used, this would ask to big matrices
        end
    end
    progressbar(ii/n)
end
fprintf(' done ! (took %2.3f sec) \n',toc)

GSL_free=GSL([GSL.free].');
figure()
show(LabGridEdges);
title('')
hold on
grid on
xlabel('x [m]')
ylabel('y [m]')
plotPath(GSL_free)

%% Graph theory with Dijkstra's algorithm
tic
fprintf('Calculating optimal path with Dijkstras algorithm ...')

% Building vertices
vx=[[GSL_free.x0].' [GSL_free.x1].']; % vertex vx = [x_start x_end]
vy=[[GSL_free.y0].' [GSL_free.y1].']; % vertex vy = [y_start y_end]
vth=[[GSL_free.th0].' [GSL_free.th1].']; % vertex vth = [th_start th_end]

% construct graph
xyth_all = unique([vx(:) vy(:) vth(:)], 'rows'); % all unique vertices
xyth_0 = [vx(:,1) vy(:,1) vth(:,1)]; % all start vertices for edge
xyth_1 = [vx(:,2) vy(:,2) vth(:,2)]; % all end vertices for edge
xyth_01= [xyth_0 xyth_1]; % start end vertices for all edges
[~,ii] = ismember(xyth_0,xyth_all,'rows'); % search idx of start idx vertix in all vertices
[~,jj] = ismember(xyth_1,xyth_all,'rows'); % search idx of end idx vertix in all vertices
pathCost=[GSL_free.pathCost].';
G = sparse(ii,jj,pathCost,length(xyth_all),length(xyth_all)); % create sparse matix, needed for graphshortestpath (uses Dijkstra's algorithm)
st_idx   = findrow_mex(xyth_all,xyth_start); % fast single row search
dest_idx = findrow_mex(xyth_all,xyth_end); % fast single row search

% Dijkstra's algorithm to find shortes path
[dist,opt_vertices,pred]=graphshortestpath(G,st_idx,dest_idx);
xyth_opt_vertices = xyth_all(opt_vertices,:);
xyth_opt_vertices_01 = [xyth_opt_vertices(1:end-1,:) xyth_opt_vertices(2:end,:)];
[~,idx_optimal_edges] =  ismember(xyth_opt_vertices_01,xyth_01,'rows');
OptPath=GSL_free(idx_optimal_edges);
fprintf(' done ! (took %2.3f sec) \n',toc)

% plot optimal path
    figure(1)
    show(LabGridEdges)
    title('Local Path Planning')
    hold on
    plotRobotPath(OptPath)
    hold off
%     l1=legend('optimal path','visited nodes','robot');
%     set(l1,'FontSize',12);dest_idx
%
%     figure(1)
%     show(LabGrid)
%     title('Local Path Planning')
%     hold on
%     plotPath(SL)
%     hold off