% function xy_opt_path = VoronoiPathPlanning()
% xy_opt_path = VoronoiPathPlanning()
% Comments about used variables :
% MISC
%
% 
clear
close all; 
clc;

%% starting point
xy_start = [1 1];
xy_dest  = [8 9];

%% area of interest (AOI)
x_min = 0; x_max = 10;
y_min = 0; y_max = 10;

num_sample = 100;

xn = rand(1,num_sample)*(x_max-x_min) + x_min;
yn = rand(1,num_sample)*(y_max-y_min) + y_min;

%% circluar obstacle
th=0:0.01:2*pi;
c_cx = 2; c_cy = 5; c_r = 1;  
xc=c_r*cos(th)+c_cx;
yc=c_r*sin(th)+c_cy;

figure(1); clf; % just creating figure for later

%% draw sampling points
plot(xn,yn,'b.');
hold on;
%keyboard

%% draw circular obstacle
plot(xc,yc,'r.');
%keyboard

%% divide region using voronoi
[vx,vy] = voronoi(xn,yn);

%% reject points outside the AOI
idx = (vx(1,:) < x_min) | (vx(2,:) < x_min);
vx(:,idx) = [];
vy(:,idx) = [];
idx = (vx(1,:) > x_max) | (vx(2,:) > x_max);
vx(:,idx) = [];
vy(:,idx) = [];
idx = (vy(1,:) < y_min) | (vy(2,:) < y_min);
vx(:,idx) = [];
vy(:,idx) = [];
idx = (vy(1,:) > y_max) | (vy(2,:) > y_max);
vx(:,idx) = [];
vy(:,idx) = [];

plot(vx,vy,'b-');
axis([x_min x_max y_min y_max]);
%keyboard

%% clear figure
clf;
%% draw circular obstacle
plot(xc,yc,'r.');
hold on;

%% remove vertrices inside the circular obstacle
vx1=vx(1,:);
vy1=vy(1,:);
r_sq = (vx1-c_cx).^2 + (vy1-c_cy).^2; % calculate distance^2 to circle
idx1 = find(r_sq < c_r^2);  % if distance^2 is less then radius^2, ind1

vx2=vx(2,:);
vy2=vy(2,:);
r_sq =(vx2 - c_cx).^2+(vy2-c_cy).^2; % calculate distance^2 to circle
idx2=find(r_sq < c_r^2);    % if distance^2 is less then radius^2, ind2

idx=union(idx1,idx2);
vx_new=vx; vx_new(:,idx)=[]; % remove found indexes from original vx
vy_new=vy; vy_new(:,idx)=[]; % remove found indexes from original vy
plot(vx_new,vy_new,'b.-');
%keyboard

%% polygon ostacle
xv = [4; 4; 8; 8; 6; 6; 4]; % start & end is the same !
yv = [4; 6; 6; 2; 2; 4; 4]; % start & end is the same !
plot(xv,yv,'k-','LineWidth',2);
%keyboard

%% redraw obstacles
clf;
plot(xc,yc,'r.'); hold on;
plot(vx_new,vy_new,'b.-');
plot(xv,yv,'k-','LineWidth',2);

%% find the points inside the polygon
vx=vx_new(1,:);
vy=vy_new(1,:);
in = inpolygon(vx,vy,xv,yv);
plot(vx(in),vy(in),'r.');
vx_new(:,in)=[];
vy_new(:,in)=[];
%keyboard

vx=vx_new(2,:);
vy=vy_new(2,:);
in = inpolygon(vx,vy,xv,yv);
plot(vx(in),vy(in),'g.');
vx_new(:,in)=[];
vy_new(:,in)=[];
%keyboard

%% draw the network
clf;
plot(xc,yc,'r.'); hold on;
plot(xv,yv, 'k-','LineWidth',2); 
plot(vx_new,vy_new,'b.-');

%% add starting point
vx_all = vx_new(:);
vy_all = vy_new(:);

dr = kron(ones(length(vx_all),1),xy_start) - [vx_all vy_all];
[min_val, min_id] = min(sum(dr.^2,2));

vx_new = [vx_new [xy_start(1); vx_all(min_id)]];
vy_new = [vy_new [xy_start(2); vy_all(min_id)]];


%% add destination point
dr = kron(ones(length(vx_all),1),xy_dest) - [vx_all vy_all];
[min_val, min_id] = min(sum(dr.^2,2));

vx_new = [vx_new [xy_dest(1); vx_all(min_id)]];
vy_new = [vy_new [xy_dest(2); vy_all(min_id)]];

%% construct graph
xy_all = unique([vx_new(:) vy_new(:)], 'rows');
dv = [vx_new(1,:); vy_new(1,:)] - [vx_new(2,:); vy_new(2,:)];
edge_dist = sqrt(sum(dv.^2));

G = sparse(size(xy_all,1),size(xy_all,1));

for kdx = 1:length(edge_dist)
    xy_s = [vx_new(1,kdx) vy_new(1,kdx)];
    idx = find(sum((xy_all-kron(ones(size(xy_all,1),1),xy_s)).^2,2)==0);
    xy_d = [vx_new(2,kdx) vy_new(2,kdx)];
    jdx = find(sum((xy_all-kron(ones(size(xy_all,1),1),xy_d)).^2,2)==0);
    G(idx,jdx) = edge_dist(kdx);
    G(jdx,idx) = edge_dist(kdx);
end

%% starting index and destination index
st_idx = find(sum((xy_all-kron(ones(size(xy_all,1),1),xy_start)).^2,2)==0);
dest_idx = find(sum((xy_all-kron(ones(size(xy_all,1),1),xy_dest)).^2,2)==0);

%% dijestra's algoritm to find shortes path
[dist,path,pred] = graphshortestpath(G,st_idx,dest_idx);
xy_opt_path = xy_all(path,:);

%% plot optimal path
plot(xy_opt_path(:,1),xy_opt_path(:,2), 'go-','LineWidth',2);
% end