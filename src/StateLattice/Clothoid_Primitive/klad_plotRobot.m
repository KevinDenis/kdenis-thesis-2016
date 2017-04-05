clearvars -except MotionPrem
if ~exist('MotionPrem', 'var')
    load('MotionPrem.mat');
end

load('SimpleRobotPlot.mat')

% clear
% close all
% clc
%  
% RobotFullBW = ~imread('RobotFull.bmp');
% robotFullGrid = robotics.BinaryOccupancyGrid(RobotFullBW,200);
% RobotFullInfo=regionprops(RobotFullBW);
% RobotFullCij=RobotFullInfo.Centroid;
% RobotFullCxy=grid2world(robotFullGrid,round([RobotFullCij(2),RobotFullCij(1)]));
% robotFullGrid.GridLocationInWorld=-RobotFullCxy;
%  
%  
%  
%  
%  
% [ii_full,jj_full]=meshgrid(1:robotFullGrid.GridSize(1),1:robotFullGrid.GridSize(2));
% ii_all_full=ii_full(:);
% jj_all_full=jj_full(:);
% occval_full =getOccupancy(robotFullGrid,[ii_all_full jj_all_full], 'grid');
% ii_occ_full=ii_all_full(occval_full==1);
% jj_occ_full=jj_all_full(occval_full==1);
% xxyy_occ_full=grid2world(robotFullGrid,[ii_occ_full jj_occ_full]);
%  
% kk=convhull(xxyy_occ_full(:,1),xxyy_occ_full(:,2),'simplify',true);
% xxyy_shell=xxyy_occ_full(kk,:);
% SimpleRobotPlot=xxyy_shell;
% figure()
% hold on
% show(robotFullGrid)
% plot(SimpleRobotPlot(:,1),SimpleRobotPlot(:,2),'b')

ii=8;

X=MotionPrem(ii).X;
Y=MotionPrem(ii).Y;
TH=MotionPrem(ii).TH;

plotRobotPath(SimpleRobotPlot,X,Y,TH)

