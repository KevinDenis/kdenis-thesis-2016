
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%                        State Latice Structure                           %
%[ x0 y0 th0 x1 y1 th1 X Y TH k dk Ltot intK PathOccXY pathCost free ID ] %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

initWorkspace


showPlot=true;

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

robotPose=[-2 2.5 pi/4];

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

tic
% progressbar('Calculating Collision Free Paths')
n=length(StateLattice);
for ii=1:n
    PathOccXY_ii=StateLattice(ii).PathOccXY;
    if any(PathOccXY_ii(:,1)<LabGrid_R.XWorldLimits(1)) || ...
       any(PathOccXY_ii(:,1)>LabGrid_R.XWorldLimits(2)) || ...
       any(PathOccXY_ii(:,2)<LabGrid_R.YWorldLimits(1)) || ...
       any(PathOccXY_ii(:,2)>LabGrid_R.YWorldLimits(2))
       StateLattice(ii).free=false; %#ok<SAGROW> % False Positive, is it not growing ! 
    else
        if any(getOccupancy(LabGrid_R,PathOccXY_ii))
            StateLattice(ii).free=false; %#ok<SAGROW> % False Positive, is it not growing ! 
        else
            StateLattice(ii).free=true; %#ok<SAGROW> % False Positive, is it not growing ! 
        end
    end
%     progressbar(ii/n)
end
toc
selectFree=[StateLattice.free].';
SL=StateLattice(selectFree);


[SL] = CleanupLooseStarts(SL);
SL_W=RotTransMotionPrem(SL,robotPose(3),robotPose(1),robotPose(2));
%%

if showPlot
    figure()
    hold on
    grid on
    xlabel('x [m]')
    ylabel('y [m]')
    axis equal
    show(LabGrid)
    title('Local Path Planning')
    plotGrid(grid_XY,robotPose)
    plotPath(SL_W)
    plotSimpleRobot(robotPose)
    l=legend('discrete grids','clothoids','circular arcs','reachable grids');
    set(l,'FontSize',12);
%     pause()
%     saveCurrentFigure   
end