
% !!!!!!!!!!!!!!!!!!!!!!!! MAN CROP NEEDED !!!!!!!!!!!!!!!!!!!!!!!!


initPlotScripts
co=get(groot,'DefaultAxesColorOrder');
gridRes=0.02;
load('LSL_cloth.mat')
[XY_occ_full] = getOccXYFromBmpRobot('RobotFull_1cm.bmp',gridRes/2);
[XY_occ_shell]= getOccXYFromBmpRobot('RobotShell_1cm.bmp',gridRes/2);

vertices = getStartEndVerticesPath(LSL);

poseStart=[0 0 0];
poseEnd=[0.5 0.1 pi/8];
poseStartEnd=[poseStart poseEnd];

idxStartEnd=findrow_mex(vertices,poseStartEnd);


%% Create empty pathh occ grid
PathOccGridEmpty=robotics.OccupancyGrid(1.8,1.6,1/gridRes);
PathOccGridEmpty.GridLocationInWorld=[-0.9 -0.8];
[ii_Path,jj_Path]=meshgrid(1:PathOccGridEmpty.GridSize(1),1:PathOccGridEmpty.GridSize(2));
ii_all_Path=ii_Path(:);
jj_all_Path=jj_Path(:);
setOccupancy(PathOccGridEmpty,[ii_all_Path jj_all_Path],zeros(size(ii_all_Path)),'grid')


%% show full and shell
FullShellRobot=robotics.OccupancyGrid(1.7,1.6,1/gridRes);
FullShellRobot.GridLocationInWorld=[-1 -0.8];
[ii_FullShellRobot,jj_FullShellRobot]=meshgrid(1:FullShellRobot.GridSize(1),1:FullShellRobot.GridSize(2));
ii_all_FullShellRobot=ii_FullShellRobot(:);
jj_all_FullShellRobot=jj_FullShellRobot(:);
setOccupancy(FullShellRobot,[ii_all_FullShellRobot jj_all_FullShellRobot],zeros(size(ii_all_FullShellRobot)),'grid')
setOccupancy(FullShellRobot,XY_occ_full,0.3*ones(size(XY_occ_full,1),1))
setOccupancy(FullShellRobot,XY_occ_shell,ones(size(XY_occ_shell,1),1))

gray=[0.7 0.7 0.7];
black=[0 0 0];

fig=figureFullScreen(1);
fig.Renderer='Painters';
subplot(1,2,1)
show(FullShellRobot)
title('')
hold on
xlabel('x [m]')
ylabel('y [m]')
plot(-0.5,0,'s','MarkerEdgeColor',gray,'MarkerFaceColor',gray,'MarkerSize',8)
plot(-0.63,0.23,'s','MarkerEdgeColor',black,'MarkerFaceColor',black,'MarkerSize',8)
l=legend('OG of full robot (used at start)','OG of robot-shell (used after start)','Location','SE');
set(l,'FontSize',26);
set(gca,'FontSize',24)
set(gca, 'box', 'off')
saveCurrentFigure('OGRobotStart');
pause()
saveCurrentFigure('OGRobotStart');
close all

fig=figureFullScreen(1);
fig.Renderer='Painters';
subplot(1,2,2)
hold on
for nn=idxStartEnd
    X=LSL(nn).X;
    Y=LSL(nn).Y;
    TH=LSL(nn).TH;
    PathOccGrid=copy(PathOccGridEmpty); % just assigning doesn't work
    for idxPath=1:1:18
        if idxPath==1 % use full occ grid (just once)
            XY_occ_kk=XY_occ_full;
        else % use shell grid (for the rest, faster ! but be sure that fine enough movement)
            XY_occ_kk=XY_occ_shell;
        end
        XY_occ_rot_trans=RotTransXY(XY_occ_kk ,TH(idxPath),X(idxPath),Y(idxPath));       
        
        occval_Path =getOccupancy(PathOccGrid,[ii_all_Path jj_all_Path], 'grid');
        ii_occ_PathOcc=ii_all_Path(occval_Path>0.4);
        jj_occ_PathOcc=jj_all_Path(occval_Path>0.4);
        
        [iijj_newPathOcc]=world2grid(PathOccGrid,XY_occ_rot_trans);
        idxUnique=~ismember(iijj_newPathOcc,[ii_occ_PathOcc jj_occ_PathOcc],'rows');
        iijj_newPathOccUnique=iijj_newPathOcc(idxUnique,:);
        XY_unique=grid2world(PathOccGrid,iijj_newPathOccUnique);

        if ~isempty(ii_occ_PathOcc)
            setOccupancy(PathOccGrid,[ii_occ_PathOcc jj_occ_PathOcc],0.3*ones(size(ii_occ_PathOcc,1),1),'grid')   
        end
        setOccupancy(PathOccGrid,XY_unique,1*ones(size(XY_unique,1),1))

        if idxPath == 18
            show(PathOccGrid)
            title('')
            hold on
            xlabel('x [m]')
            ylabel('y [m]')
            plot(-0.5,0,'s','MarkerEdgeColor',gray,'MarkerFaceColor',gray,'MarkerSize',8)
            plot(0.63,0.35,'s','MarkerEdgeColor',black,'MarkerFaceColor',black,'MarkerSize',8)
            plot(X(1:idxPath),Y(1:idxPath),'Color',co(1,:),'Linewidth',3);
            plot(X,Y,'--','Color',co(1,:),'Linewidth',2);
            text(-0.25,0.70,'Path\_Start  : [0    0      0°   ]','FontSize',24)
            text(-0.25,0.60,'Path\_Dest  : [0.5 0.1 22.5°]','FontSize',24)
            text(-0.25,0.50,['Path\_ID  : ',num2str(LSL(nn).ID)],'FontSize',24)
            text(-0.25,0.40,['Path\_Idx : ',num2str(idxPath)],'FontSize',24)
            l=legend('Previously visited cells','Newly visited cells','Travelled path','Total path','Location','SE');
            set(l,'FontSize',26);
            set(gca,'FontSize',24)
            set(gca,'FontSize',24)
            drawnow
        end
        setOccupancy(PathOccGrid,XY_occ_rot_trans,ones(size(XY_occ_rot_trans,1),1))
        if ~isempty(ii_occ_PathOcc)
            setOccupancy(PathOccGrid,[ii_occ_PathOcc jj_occ_PathOcc],ones(size(ii_occ_PathOcc,1),1),'grid')   
        end
    end
end
% saveCurrentFigure('OGRobotPath');