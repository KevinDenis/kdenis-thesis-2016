% Notes :
%   * better to have a res 2x of the robot compared to the final grid

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%                        State Latice Structure                           %
%[ x0 y0 th0 x1 y1 th1 X Y TH k dk Ltot intK PathOccXY pathCost free ID ] %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%#ok<*UNRCH>

initWorkspace

showPlot=false;

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

PathOccGridEmpty=robotics.BinaryOccupancyGrid(12,12,50);
PathOccGridEmpty.GridLocationInWorld=[-6 -6];

[ii_full,jj_full]=meshgrid(1:robotFullGrid.GridSize(1),1:robotFullGrid.GridSize(2));
ii_all_full=ii_full(:);
jj_all_full=jj_full(:);
occval_full =getOccupancy(robotFullGrid,[ii_all_full jj_all_full], 'grid');
ii_occ_full=ii_all_full(occval_full==1);
jj_occ_full=jj_all_full(occval_full==1);
XY_occ_full=grid2world(robotFullGrid,[ii_occ_full jj_occ_full]);

[ii_shell,jj_shell]=meshgrid(1:robotShellGrid.GridSize(1),1:robotShellGrid.GridSize(2));
ii_all_shell=ii_shell(:);
jj_all_shell=jj_shell(:);
occval_shell =getOccupancy(robotShellGrid,[ii_all_shell jj_all_shell], 'grid');
ii_occ_shell=ii_all_shell(occval_shell==1);
jj_occ_shell=jj_all_shell(occval_shell==1);
XY_occ_shell=grid2world(robotShellGrid,[ii_occ_shell jj_occ_shell]);

% figure(2)
% setOccupancy(PathOccGrid,[xxyy_occ_shell(:,1) xxyy_occ_shell(:,2)],ones(length(xxyy_occ_shell),1))
% show(PathOccGrid)

if showPlot
    figure(2) 
else
    progressbar('Calculating Occupancy Grid of Local State Lattice')
end

n=length(StateLattice);
for nn=1:n
    PathOccGrid=copy(PathOccGridEmpty);
    X=StateLattice(nn).X;
    Y=StateLattice(nn).Y;
    TH=StateLattice(nn).TH;
  
    for kk=1:1:length(X)
        if kk==1 % use full occ grid, just once
            X_occ=XY_occ_full(:,1);
            Y_occ=XY_occ_full(:,2); 
        else % use shell grid
            X_occ=XY_occ_shell(:,1);
            Y_occ=XY_occ_shell(:,2);
        end
        XY_occ_rot_trans=RotTransXY([X_occ Y_occ],TH(kk),X(kk),Y(kk));
        setOccupancy(PathOccGrid,XY_occ_rot_trans,ones(size(XY_occ_rot_trans,1),1))
    end
    
    [ii_PathOccGrid,jj_Path]=meshgrid(1:PathOccGrid.GridSize(1),1:PathOccGrid.GridSize(2));
    ii_all_Path=ii_PathOccGrid(:);
    jj_all_Path=jj_Path(:);
    occval_Path =getOccupancy(PathOccGrid,[ii_all_Path jj_all_Path], 'grid');
    ii_occ_Path=ii_all_Path(occval_Path==1);
    jj_occ_Path=jj_all_Path(occval_Path==1);
    PathOccXY=grid2world(PathOccGrid,[ii_occ_Path jj_occ_Path]);
   
    StateLattice(nn).PathOccXY=PathOccXY; %#ok<SAGROW> % False Positive, is it not growing ! 
        
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