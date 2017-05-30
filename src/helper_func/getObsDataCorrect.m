initWorkspace
load('idxGood.mat')
% plot(obs_out(:,1),obs_out(:,2),'ko')
% plot(vx_all,vy_all,'bo')


%   Example 1:
%       % Self-intersecting polygon

%       in = inpolygon(x,y,xv,yv);
%       plot(xv,yv,x(in),y(in),'.r',x(~in),y(~in),'.b')


% figure()
% 
[~,obs_out] = getXYOccFormBmpMap('RobotLabo_Lift_ObsOutCopy.bmp',0.02);
obs_out=obs_out(idxGood,:);
obs_out_x=obs_out(:,1);
obs_out_y=obs_out(:,2);


[~,obs_in1] = getXYOccFormBmpMap('RobotLabo_Lift_ObsIn1.bmp',0.02);
kkObs1=unique(convhull(obs_in1(:,1),obs_in1(:,2),'simplify', true));
obs_in1=obs_in1(kkObs1,:);
[obs_in1_x, obs_in1_y]=poly2ccw(obs_in1(:,1),obs_in1(:,2));
obs_in1_x=flip(obs_in1_x);
obs_in1_x=[obs_in1_x;obs_in1_x(1)];
obs_in1_y=flip(obs_in1_y);
obs_in1_y=[obs_in1_y;obs_in1_y(1)];

[~,obs_in2] = getXYOccFormBmpMap('RobotLabo_Lift_ObsIn2.bmp',0.02);
kkObs2=unique(convhull(obs_in2(:,1),obs_in2(:,2),'simplify', true));
obs_in2=obs_in2(kkObs2,:);
[obs_in2_x, obs_in2_y]=poly2ccw(obs_in2(:,1),obs_in2(:,2));
obs_in2_x=flip(obs_in2_x);
obs_in2_x=[obs_in2_x;obs_in2_x(1)];
obs_in2_y=flip(obs_in2_y);
obs_in2_y=[obs_in2_y;obs_in2_y(1)];

obs_x=[obs_out_x; nan; obs_in1_x; nan; obs_in2_x];
obs_y=[obs_out_y; nan; obs_in1_y; nan; obs_in2_y];


x = rand(10000,1)*25; 
y = rand(10000,1)*25;
 
[in,~] = inpolygon(x,y,obs_x,obs_y);
figure()
hold on
axis equal
plot(obs_x,obs_y,'b');
plot(x(in),y(in),'ok');

%% method 2

LabBW = imread(bmp);



[~,LabGridEdges,XY_occ_lab_edges] = getMapEdgeXYOccFormBmp(bmp,0.02);

[x_obs,y_obs]=sortPointMinDist(XY_occ_lab_edges(:,1),XY_occ_lab_edges(:,2));
obs=[x_obs,y_obs];



xy_start = [4.5 10];
xy_dest = [19.75 16.25];


[LabGrid,XY_corners] = getXYOccFormBmpMap('RobotLabo_Lift_Corners.bmp',0.02);

idx=find(ismember(obs,XY_corners,'rows')==1);
XY_corners_CW=obs(idx,:);


figure(1)
hold on;
plot(x_obs,y_obs, 'k');
plot(XY_corners_CW(:,1),XY_corners_CW(:,2), 'g');
% plot(vx,vy,'b.-'); hold on;




 
% show(labgrid)
% 
% 
% 
% [labgrid,obs_out] = getXYOccFormBmpMap('RobotLabo_Lift_ObsOutCopy.bmp',0.02);
% obs_x=obs_out(:,1);
% obs_y=obs_out(:,2);
% figure()
% scatter(obs_x,obs_y,'k','filled')
% axis equal
% hold on
% idxGood=zeros(length(obs_out),1)
% for kk=1:length(obs_out)
%     [x,y]=ginput(1);
%     dr = kron(ones(length(obs_out),1),[x y]) - obs_out;
%     [min_val, min_id] = min(sum(dr.^2,2));
%     scatter(obs_x(min_id),obs_y(min_id),'b','filled')
%     idxGood(kk)=min_id;
% end