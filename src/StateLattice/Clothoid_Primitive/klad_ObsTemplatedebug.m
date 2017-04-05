
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%                        State Latice Structure                           %
%[ x0 y0 th0 x1 y1 th1 X Y TH k dk Ltot intK PathOccXY pathCost free ID ] %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clearvars -except StateLattice MotionPrem ObstacleTable XY_all
% if ~exist('StateLattice', 'var')
%     disp('loading StateLattice.mat')
%     load('StateLattice.mat')
% end
close all
clc
addpath('ClothoidG1fitting');
%#ok<*SAGROW,*UNRCH>

dx=0.25;
dth=pi/8;
showPlot=false;

% Import Image
LabBW = ~imread('ComplicatedLab.bmp');
LabGrid = robotics.BinaryOccupancyGrid(LabBW,100);
LabGrid.GridLocationInWorld=[-5 -5];

% figure()
% hold on
% show(LabGrid)
% [xStart,yStart]=ginput(1);
% scatter(xStart,yStart,'*r')
% [xOrient,yOrient]=ginput(1);
% plot([xStart xOrient],[yStart yOrient],'*-r')
% thStart=atan2(yOrient-yStart,xOrient-xStart);
% xStart=round(xStart/dx)*dx;
% yStart=round(yStart/dx)*dx;
% thStart=round(thStart/dth)*dth;
% robotPose=[xStart yStart thStart];

robotPose=[2 3 pi];

[ii_lab,jj_lab]=meshgrid(1:LabGrid.GridSize(1),1:LabGrid.GridSize(2));
ii_all_lab=ii_lab(:);
jj_all_lab=jj_lab(:);
occval_lab =getOccupancy(LabGrid,[ii_all_lab jj_all_lab], 'grid');
ii_occ_lab=ii_all_lab(occval_lab==1);
jj_occ_lab=jj_all_lab(occval_lab==1);
XY_occ_lab=grid2world(LabGrid,[ii_occ_lab jj_occ_lab]);


XY_occ_lab_R=TransRotXY(XY_occ_lab,-robotPose(3),-robotPose(1),-robotPose(2));


XY_occ_lab_R(abs(XY_occ_lab_R(:,1))>5,:)=[];
XY_occ_lab_R(abs(XY_occ_lab_R(:,2))>5,:)=[];

LabGrid_R = robotics.BinaryOccupancyGrid(10,10,100);
LabGrid_R.GridLocationInWorld=[-5 -5];
setOccupancy(LabGrid_R,XY_occ_lab_R,ones(size(XY_occ_lab_R,1),1))
inflate(LabGrid_R,0.001)

[ii_lab_R,jj_lab_R]=meshgrid(1:LabGrid_R.GridSize(1),1:LabGrid_R.GridSize(2));
ii_all_lab_R=ii_lab_R(:);
jj_all_lab_R=jj_lab_R(:);
occval_lab_R =getOccupancy(LabGrid_R,[ii_all_lab_R jj_all_lab_R], 'grid');
ii_occ_lab_R=ii_all_lab(occval_lab_R==1);
jj_occ_lab_R=jj_all_lab(occval_lab_R==1);
XY_occ_lab_R=grid2world(LabGrid_R,[ii_occ_lab_R jj_occ_lab_R]);


progressbar('Calculating Collision Free Paths')
n=length(XY_occ_lab_R);
toDelete=zeros(1);
kk=0;
for ii=1:n
    xy_occ_ii=XY_occ_lab_R(ii,:);
    idx=findrow_mex(XY_all,xy_occ_ii);
    if ~isnan(idx)
        paths_ii=[ObstacleTable(idx).paths];
        kk=kk(end)+[1:length(paths_ii)];
        toDelete(kk)=paths_ii;
    end
    progressbar(ii/n)
end




% progressbar('Calculating Collision Free Paths')
% n=length(StateLattice);
% for ii=1:n
%     PathOccXY_ii=StateLattice(ii).PathOccXY;
%     if any(PathOccXY_ii(:,1)<LabGrid_R.XWorldLimits(1)) || ...
%        any(PathOccXY_ii(:,1)>LabGrid_R.XWorldLimits(2)) || ...
%        any(PathOccXY_ii(:,2)<LabGrid_R.YWorldLimits(1)) || ...
%        any(PathOccXY_ii(:,2)>LabGrid_R.YWorldLimits(2))
%        StateLattice(ii).free=false;
%     else
%         if any(getOccupancy(LabGrid_R,PathOccXY_ii))
%             StateLattice(ii).free=false;
%         else
%             StateLattice(ii).free=true;
%         end
%     end
% %     progressbar(ii/n)
% end
% 
% selectFree=[StateLattice.free].';
% SL=StateLattice(selectFree);
% tic
% [SL] = CleanupLooseStarts(SL);
% toc
% %%
% figure()
% show(LabGrid_R)
% hold on
% grid on
% xlabel('x [m]')
% ylabel('y [m]')
% axis equal
% plotPath(SL)