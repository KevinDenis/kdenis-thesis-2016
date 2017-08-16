function XY_occ_obs=getXYobsFromBMP(obs_circle_bmp)
obs_circle_BW = ~imread(obs_circle_bmp);
obsGrid = robotics.BinaryOccupancyGrid(obs_circle_BW,100);
[ii,jj]=meshgrid(1:obsGrid.GridSize(1),1:obsGrid.GridSize(2));
ii_all=ii(:);
jj_all=jj(:);
occval =getOccupancy(obsGrid,[ii_all jj_all], 'grid');
ii_occ=ii_all(occval==1);
jj_occ=jj_all(occval==1);
XY_occ_obs=grid2world(obsGrid,[ii_occ jj_occ]);
% robotGrid.GridLocationInWorld=-RobotCxy;
CenterOfMassXY = [mean(XY_occ_obs(:,1)) mean(XY_occ_obs(:,2))];
obsGrid.GridLocationInWorld=-CenterOfMassXY;
XY_occ_obs=grid2world(obsGrid,[ii_occ jj_occ]);
end