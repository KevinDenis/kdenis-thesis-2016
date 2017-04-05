clearvars -except StateLattice MotionPrem
% if ~exist('StateLattice', 'var')
%     disp('loading StateLattice.mat')
%     load('StateLattice.mat')
% end
co=get(gca,'ColorOrder'); % get default color for plot
close all
clc
addpath('ClothoidG1fitting');
%#ok<*SAGROW>
%#ok<*UNRCH>

showPlot=false;

% Import Image
LabBW = ~imread('ComplicatedLab.bmp');
LabGrid = robotics.BinaryOccupancyGrid(LabBW,100);
LabGrid.GridLocationInWorld=[-5 -5];
% show(LabGrid)

progressbar('Calculating Collision Free Paths')

n=length(StateLattice);
for ii=1:n
    PathOccXY_ii=StateLattice(ii).PathOccXY;
    if any(PathOccXY_ii(:,1)<LabGrid.XWorldLimits(1)) || ...
       any(PathOccXY_ii(:,1)>LabGrid.XWorldLimits(2)) || ...
       any(PathOccXY_ii(:,2)<LabGrid.YWorldLimits(1)) || ...
       any(PathOccXY_ii(:,2)>LabGrid.YWorldLimits(2))
       StateLattice(ii).free=false;
    else
        if any(getOccupancy(LabGrid,PathOccXY_ii))
            StateLattice(ii).free=false;
        end
    end
    progressbar(ii/n)
end
% save('StateLattice.mat','StateLattice')
%}