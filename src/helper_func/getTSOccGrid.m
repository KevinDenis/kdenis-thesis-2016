function [TS_occ,ConvHull_kk]=getTSOccGrid(t_impact,Path,RobotHull,XY_occ_obs,C_xy_start_obs,V_c_obs)
co=get(groot,'DefaultAxesColorOrder');
Path_S = Path.S;
len_Path_S = length(Path_S);
Path_XYTH = [[Path.X] [Path.Y] [Path.TH]];

lenObs=size(XY_occ_obs,1);
C_xy_start_obs_rep = repmat(C_xy_start_obs,lenObs,1);


len_t_impact = length(t_impact);
TS_all=getAllComb(t_impact,Path_S);
TS_Occ_all = [TS_all false(size(TS_all,1),1)];


for ii = 1:len_t_impact
    t_ii=t_impact(ii);
    for jj = 1:len_Path_S
        Path_XYTH_jj=Path_XYTH(jj,:);
        RobotHull_jj=RotTransXY(RobotHull,Path_XYTH_jj(3),Path_XYTH_jj(1),Path_XYTH_jj(2));
        XY_occ_obs_t_ii=XY_occ_obs+C_xy_start_obs_rep+repmat(t_ii*V_c_obs,lenObs,1);
        collision=any(InPolygon(XY_occ_obs_t_ii(:,1),XY_occ_obs_t_ii(:,2),RobotHull_jj(:,1),RobotHull_jj(:,2)));
        kk=jj+(ii-1)*len_Path_S;
        TS_Occ_all(kk,3)=collision;
%         if collision
%         if rem(jj,10) == 0 && rem(ii,10) == 0
%         clf
%         figure(1)
%         hold on
%         if collision
%             plotRoboticWheelchair(Path_XYTH_jj,co(2,:))
%         else
%             plotRoboticWheelchair(Path_XYTH_jj)
%         end
%         plotPath(Path)
%         plot(XY_occ_obs_t_ii(:,1),XY_occ_obs_t_ii(:,2),'.','Color',co(2,:))
%         axis([-0.5 2.5 -0.5 2.5])
%         hold off
%         drawnow
%         end
%         end
    end
end
Occ_all=TS_Occ_all(:,3)==1;
TS_all = TS_Occ_all(:,1:2);

TS_occ = TS_all(Occ_all,:);

ConvHull_kk = convhull(TS_occ(:,1),TS_occ(:,2),'simplify', true);

end