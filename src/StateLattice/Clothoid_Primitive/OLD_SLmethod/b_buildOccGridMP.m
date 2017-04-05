clearvars -except MotionPrem
if ~exist('MotionPrem', 'var')
    load('MotionPrem.mat');
end
co=get(gca,'ColorOrder'); % get default color for plot
close all
clc
addpath('ClothoidG1fitting');
%#ok<*SAGROW>
%#ok<*UNRCH>

% notes :
% better to have a res 2x of the robot compared to the final grid

showPlot=true;


% Import Image

RobotFullBW = ~imread('RobotFullRed.bmp');
robotFullGrid = robotics.BinaryOccupancyGrid(RobotFullBW,50);
RobotFullInfo=regionprops(RobotFullBW);
RobotFullCij=RobotFullInfo.Centroid;
RobotFullCxy=grid2world(robotFullGrid,round([RobotFullCij(2),RobotFullCij(1)]));
robotFullGrid.GridLocationInWorld=-RobotFullCxy;

RobotShellBW = ~imread('RobotShellRed.bmp');
RobotShellInfo=regionprops(RobotShellBW);
RobotShellCij=RobotShellInfo.Centroid;
robotShellGrid = robotics.BinaryOccupancyGrid(RobotShellBW,50);
robotShellGrid.GridLocationInWorld=-RobotFullCxy;


% figure(1)
% subplot(2,1,1)
% show(robotFullGrid); hold on
% scatter(0,0,400,'*r')
% subplot(2,1,2)
% show(robotShellGrid); hold on
% scatter(0,0,400,'*r')


PathOccGrid=robotics.BinaryOccupancyGrid(6,6,20);
PathOccGrid.GridLocationInWorld=[-3 -3];




[ii_full,jj_full]=meshgrid(1:robotFullGrid.GridSize(1),1:robotFullGrid.GridSize(2));
ii_all_full=ii_full(:);
jj_all_full=jj_full(:);
occval_full =getOccupancy(robotFullGrid,[ii_all_full jj_all_full], 'grid');
ii_occ_full=ii_all_full(occval_full==1);
jj_occ_full=jj_all_full(occval_full==1);
xxyy_occ_full=grid2world(robotFullGrid,[ii_occ_full jj_occ_full]);


[ii_shell,jj_shell]=meshgrid(1:robotShellGrid.GridSize(1),1:robotShellGrid.GridSize(2));
ii_all_shell=ii_shell(:);
jj_all_shell=jj_shell(:);
occval_shell =getOccupancy(robotShellGrid,[ii_all_shell jj_all_shell], 'grid');
ii_occ_shell=ii_all_shell(occval_shell==1);
jj_occ_shell=jj_all_shell(occval_shell==1);
xxyy_occ_shell=grid2world(robotShellGrid,[ii_occ_shell jj_occ_shell]);

% figure(2)
% setOccupancy(PathOccGrid,[xxyy_occ_shell(:,1) xxyy_occ_shell(:,2)],ones(length(xxyy_occ_shell),1))
% show(PathOccGrid)


if showPlot
    figure(2)
else
    progressbar('Calculating Occupancy Grid of Motion Primitives')
end

n=length(MotionPrem);
for nn=1:n
    PathOccGrid=robotics.BinaryOccupancyGrid(6,6,20);
    PathOccGrid.GridLocationInWorld=[-3 -3];
    X=MotionPrem(nn).X;
    Y=MotionPrem(nn).Y;
    TH=MotionPrem(nn).TH;
%     
    for kk=1:1:length(X)
        if kk==1 % use full occ grid
            xx_occ=xxyy_occ_full(:,1);
            yy_occ=xxyy_occ_full(:,2); 
        else % use shell grid
            xx_occ=xxyy_occ_shell(:,1);
            yy_occ=xxyy_occ_shell(:,2);
        end
        [xx_occ_rot,yy_occ_rot]=Rotate0Theta(xx_occ,yy_occ,TH(kk));
        xx_occ_rot_trans=xx_occ_rot+X(kk);
        yy_occ_rot_trans=yy_occ_rot+Y(kk);
        setOccupancy(PathOccGrid,[xx_occ_rot_trans yy_occ_rot_trans],ones(size(xx_occ_rot_trans)))
    end
    
    [ii_PathOccGrid,jj_Path]=meshgrid(1:PathOccGrid.GridSize(1),1:PathOccGrid.GridSize(2));
    ii_all_Path=ii_PathOccGrid(:);
    jj_all_Path=jj_Path(:);
    occval_Path =getOccupancy(PathOccGrid,[ii_all_Path jj_all_Path], 'grid');
    ii_occ_Path=ii_all_Path(occval_Path==1);
    jj_occ_Path=jj_all_Path(occval_Path==1);
    PathOccXY=grid2world(PathOccGrid,[ii_occ_Path jj_occ_Path]);
   
    MotionPrem(nn).PathOccXY=PathOccXY;
        
    if showPlot
%         if IsNear(MotionPrem(nn).th0,pi/2,1e-4) 
            hold on
            show(PathOccGrid)
            plot(X(1:kk),Y(1:kk),'r','LineWidth',3)
            hold off
            drawnow
%         end
    else
        progressbar(nn/n)
    end
 
    
   

end

% save('MotionPrem.mat','MotionPrem')
%}