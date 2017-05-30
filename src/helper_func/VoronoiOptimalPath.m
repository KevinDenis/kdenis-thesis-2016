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


function xy_opt_path= VoronoiOptimalPath(xy_start, xy_dest)
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

dr = kron(ones(length(vx_all),1),xy_start) - [vx_all vy_all];
[~, min_id] = min(sum(dr.^2,2));

vx = [vx [xy_start(1); vx_all(min_id)]];
vy = [vy [xy_start(2); vy_all(min_id)]];

% add destination point and find closest Voronoi Node
dr = kron(ones(length(vx_all),1),xy_dest) - [vx_all vy_all];
[~, min_id] = min(sum(dr.^2,2));

vx = [vx [xy_dest(1); vx_all(min_id)]];
vy = [vy [xy_dest(2); vy_all(min_id)]];

% construct of graph
xy_all = unique([vx(:) vy(:)], 'rows');
dv = [vx(1,:); vy(1,:)] - [vx(2,:); vy(2,:)];
edge_dist = sqrt(sum(dv.^2));

xy_s = [vx(1,:).' vy(1,:).'];
ii = findVector(xy_all,xy_s);
xy_d = [vx(2,:).' vy(2,:).'];
jj = findVector(xy_all,xy_d);

G = sparse([ii jj],[jj ii],[edge_dist;edge_dist],length(xy_all),length(xy_all));

% starting index and destination index
st_idx = findVector(xy_all,xy_start);

dest_idx = findVector(xy_all,xy_dest);

% Dijkstra's algoritm to find shortes path
[~,path,~] = graphshortestpath(G,st_idx,dest_idx);
xy_opt_path = xy_all(path,:); % select all collums from choses indexes

xy_opt_path=LinInterpToRes(xy_opt_path,0.01);
xy_opt_path=KeepToRes(xy_opt_path,0.5);
xy_opt_path=unique(round(round(xy_opt_path/0.25)*0.25,2),'rows','stable');
% % plot voronoi diagram optimal path
figure(1)
plot(x_obs,y_obs, 'k'); hold on;
plot(vx,vy,'b.-'); hold on;
plot(xy_opt_path(:,1),xy_opt_path(:,2), 'go-','LineWidth',2);
hold off;

end

