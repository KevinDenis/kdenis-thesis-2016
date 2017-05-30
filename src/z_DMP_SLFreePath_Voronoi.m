%=========================================================================%
%                                                                         %
%                    Build State Lattice With Grid                        %
%                    -----------------------------                        %
%                                                                         %
%                                                                         %
% Overview :                                                              %
%   *                                                                     %
%   *                                                                     %
%   *                                                                     %
%                                                                         %
% Kevin DENIS, KU Leuven, 2016-17                                         %
% Master Thesis: Path planning algorithm for semi-autonomous              %
%                mobile robots with fast and accurate collision checking  %
%                                                                         %
%=========================================================================%

%=========================================================================%
%                                                                         %
%                        State Latice Structure                           %
%                        ----------------------                           %
%             [ x0 y0 th0 x1 y1 th1 X Y TH S K k dk Ltot                  %
%              intK PathOccXY pathCost free idxBlocked ID]                %
%                                                                         %
%                        Obstacle Table Structure                         %
%                        ------------------------                         %
%                     [ x y (path)ID (path) blockedIdx]                   %
%                                                                         %
%=========================================================================%

%% 2D planner
initWorkspace
load('SL_DMP.mat')
xy_start=[5.5 8];
xy_end=[19.75 16.25];
xyth_start = [xy_start pi/2];
xyth_end = [xy_end 0];
path= VoronoiOptimalPath(xy_start, xy_end);
[map,LabGridEdges,XY_occ_lab] = getMapEdgeXYOccFormBmp('RobotLabo_Lift_Black.bmp',0.02);
LabGrid = map;

%% State Lattice Creation on 2D paths & calculating free paths

x_sift_vec=path(:,1);
y_sift_vec=path(:,2);

growthFactor=length(x_sift_vec);
lengthLSL=length(StateLattice);
lengthGSL=growthFactor*lengthLSL;
GlobalStateLattice=StateLattice(1);
GlobalStateLattice(lengthGSL)=StateLattice(end);
GlobalStateLattice=GlobalStateLattice.';

kk=1:lengthLSL;
for ii=1:growthFactor
    GlobalStateLattice(kk)=StateLattice;
   kk=kk+lengthLSL;
end

counter=0;
progressbar('Building State Lattice')
kk=1:length(StateLattice);
for ll=1:length(x_sift_vec)
    x_shift=x_sift_vec(ll);
    y_shift=y_sift_vec(ll);
    for ii=kk
        GlobalStateLattice(ii).x0=GlobalStateLattice(ii).x0+x_shift;
        GlobalStateLattice(ii).y0=GlobalStateLattice(ii).y0+y_shift;
        GlobalStateLattice(ii).x1=GlobalStateLattice(ii).x1+x_shift;
        GlobalStateLattice(ii).y1=GlobalStateLattice(ii).y1+y_shift;
        GlobalStateLattice(ii).X=GlobalStateLattice(ii).X+x_shift;
        GlobalStateLattice(ii).Y=GlobalStateLattice(ii).Y+y_shift;
        GlobalStateLattice(ii).PathOccXY(:,1)=GlobalStateLattice(ii).PathOccXY(:,1)+x_shift;
        GlobalStateLattice(ii).PathOccXY(:,2)=GlobalStateLattice(ii).PathOccXY(:,2)+y_shift;
        counter=counter+1;
        GlobalStateLattice(ii).ID=counter;
        progressbar(counter/lengthGSL)
    end
    kk=kk+length(StateLattice);
end

progressbar('Calculating Collision Free Paths')

n=length(GlobalStateLattice);
for ii=1:n
    PathOccXY_ii=GlobalStateLattice(ii).PathOccXY;
    PathOccXY_ii=PathOccXY_ii(:,1:2);
    if any(PathOccXY_ii(:,1)<(map.XWorldLimits(1))+0.5) || ...
       any(PathOccXY_ii(:,1)>(map.XWorldLimits(2))-0.5)|| ...
       any(PathOccXY_ii(:,2)<(map.YWorldLimits(1))+0.5) || ...
       any(PathOccXY_ii(:,2)>(map.YWorldLimits(2))-0.5)
       GlobalStateLattice(ii).free=false;
    else
        if any(getOccupancy(map,PathOccXY_ii))
            GlobalStateLattice(ii).free=false;
        end
    end
    progressbar(ii/n)
end

selectFree=[GlobalStateLattice.free].';
SL=GlobalStateLattice(selectFree);
figure()
show(map);
title('')
hold on
grid on
xlabel('x [m]')
ylabel('y [m]')
plotPath(SL)

%% Graph theory with dijckstra

% Small Correction on path Cost
for ii=1:length(SL)
    if SL(ii).x0 == SL(ii).x1 && SL(ii).y0 == SL(ii).y1
        SL(ii).pathCost=2*abs((SL(ii).th1-SL(ii).th0));
    end
end

% Building vertices
vx=[[SL.x0].' [SL.x1].'];
vy=[[SL.y0].' [SL.y1].'];
vth=[[SL.th0].' [SL.th1].'];

% construct graph
xyth_all = unique([vx(:) vy(:) vth(:)], 'rows');
xyth_0 = [vx(:,1) vy(:,1) vth(:,1)];
xyth_1 = [vx(:,2) vy(:,2) vth(:,2)];

tic
ii =  findVector(xyth_all,xyth_0);
jj =  findVector(xyth_all,xyth_1);
toc

pathCost=[SL.pathCost].';

G = sparse(ii,jj,pathCost,length(xyth_all),length(xyth_all));




st_idx = findVector(xyth_all,xyth_start);
dest_idx = findVector(xyth_all,xyth_end);
% dest_idx = find(ismembertol(xyth_all,xyth_end,1e-2,'byRow',true)==1);
% dest_idx=dest_idx(1);


voxel=[[SL.x0].',[SL.y0].',[SL.th0].',[SL.x1].',[SL.y1].',[SL.th1].'];

% dijestra's algoritm to find shortes path
[dist,path,pred]=graphshortestpath(G,st_idx,dest_idx);
xyth_opt_path = xyth_all(path,:);
optimal_edges = zeros(size(xyth_opt_path,1)-1,1);
for ii=1:size(xyth_opt_path,1)-1
    xyth_sd=[xyth_opt_path(ii,:) xyth_opt_path(ii+1,:)];
    optimal_edges(ii)=findVector(voxel,xyth_sd);
end
% toc
OptPath=SL(optimal_edges);

% plot optimal path
    figure(1)
    show(LabGrid)
    title('Local Path Planning')
    hold on
    plotRobotPath(OptPath)
    hold off
%     l1=legend('optimal path','visited nodes','robot');
%     set(l1,'FontSize',12);dest_idx
%}

%     figure(1)
%     show(LabGrid)
%     title('Local Path Planning')
%     hold on
%     plotPath(SL)
%     hold off
