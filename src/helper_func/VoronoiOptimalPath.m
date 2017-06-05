%=========================================================================%
%                                                                         %
%                      Voronoi Optimal Path Planning                      %
%                      -----------------------------                      %
%                                                                         %
% User settings :                                                         %
%   * TODO                                                                %
%                                                                         %
% Overview :                                                              %
%   * TODO                                                                %
%                                                                         %
% TODO :                                                                  %
%   * TODO                                                                %
%                                                                         %
% Kevin DENIS, KU Leuven, 2016-17                                         %
% Master Thesis: Path planning algorithm for semi-autonomous              %
%                mobile robots with fast and accurate collision checking  %
%                                                                         %
%=========================================================================%

function xy_opt_path= VoronoiOptimalPath(xy_start, xy_dest,pathRes,maxNodeDist,showPlot)
tic
fprintf('Calculating shortest path based on voronoi diagram ... ');
load('XY_corners_CW.mat')
obs=LinInterpToRes(XY_corners_CW,0.1);
x_obs=obs(:,1);
y_obs=obs(:,2);

warning off
[vx_tmp,vy_tmp] = voronoi(obs(:,1),obs(:,2));
warning on

%% redraw obstacles
vx=vx_tmp;
vy=vy_tmp;

vx_start=vx(1,:);
vy_start=vy(1,:);
[~, ~,in] = InPolygon(vx_start,vy_start,x_obs,y_obs);
vx(:,~in)=[];
vy(:,~in)=[];

vx_end=vx(2,:);
vy_end=vy(2,:);
[~, ~,in] = InPolygon(vx_end,vy_end,x_obs,y_obs);
vx(:,~in)=[];
vy(:,~in)=[];

% add starting point and find closest Voronoi Node
vx_all = vx(:);
vy_all = vy(:);
dr = DNorm2([vx_all vy_all]-repmat(xy_start,size(vx_all)),2); % distance start node to all other nodes
[~, idx_start] = min(dr); % pick closest node to start node
vx = [vx [xy_start(1); vx_all(idx_start)]]; % make an edge going from start node to closest node
vy = [vy [xy_start(2); vy_all(idx_start)]]; % make an edge going from start node to closest node

% add destination point and find closest Voronoi Node
dr = DNorm2( [vx_all vy_all] - repmat(xy_dest,size(vx_all)),2); % distance end node to all other nodes
[~, idx_end] = min(dr); % pick closest node to end node
vx = [vx [xy_dest(1); vx_all(idx_end)]]; % make an edge going from closest node to end nodes
vy = [vy [xy_dest(2); vy_all(idx_end)]]; % make an edge going from closest node to end nodes

% construct of graph
xy_all = unique([vx(:) vy(:)], 'rows');
path_cost = DNorm2( [vx(1,:); vy(1,:)]  - [vx(2,:); vy(2,:)]); % path cost = dist. Repeated twice because start --> end == end --> start cost (see further)
xy_0 = [vx(1,:).' vy(1,:).'];
xy_1 = [vx(2,:).' vy(2,:).'];
[~,ii] = ismember(xy_0,xy_all,'rows');
[~,jj] = ismember(xy_1,xy_all,'rows');
G = sparse([ii jj],[jj ii],[path_cost;path_cost],length(xy_all),length(xy_all));

% starting index and destination index
st_idx = findrow_mex(xy_all,xy_start);
dest_idx = findrow_mex(xy_all,xy_dest);

% Dijkstra's algoritm to find shortes path
[~,path,~] = graphshortestpath(G,st_idx,dest_idx);
xy_opt_path = xy_all(path,:); % select all collums from choses indexes

xy_opt_path=LinInterpToRes(xy_opt_path,0.01);
xy_opt_path=KeepToRes(xy_opt_path,maxNodeDist);
xy_opt_path=unique(round(round(xy_opt_path/pathRes)*pathRes,2),'rows','stable');
% % plot voronoi diagram optimal path
vx_plot=[vx;nan(1,size(vx,2))];
vx_plot=vx_plot(:);
vy_plot=[vy;nan(1,size(vy,2))];
vy_plot=vy_plot(:);
fprintf(' done ! (took %2.3f sec) \n',toc)
if showPlot
    figure()
    hold on;
    plot(XY_corners_CW(:,1),XY_corners_CW(:,2), 'k');
    plot(vx_plot,vy_plot,'b.-');
    plot(xy_opt_path(:,1),xy_opt_path(:,2), 'go-','LineWidth',2);
    hold off;
    l=legend('Obstacle','Voronoi Diagram','Optimal Path','Location','SE');
    xlabel('x [m]')
    ylabel('y [m]')
    set(gca,'FontSize',14)
    set(l,'FontSize',16);
    set(gca,'FontSize',14)
    set(gca, 'box', 'off')
end
end

