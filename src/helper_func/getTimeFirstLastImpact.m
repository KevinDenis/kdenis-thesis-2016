
function [firstImpactTime,lastImpactTime]=getTimeFirstLastImpact(PathOccGrid_XY,XY_occ_obs,C_xy_start_obs,V_c_obs,t)
co=get(groot,'DefaultAxesColorOrder');

hash = randn(size(PathOccGrid_XY,2),1);
PathOccGrid_XY_hash = round(PathOccGrid_XY*hash,6);

lenObs=size(XY_occ_obs,1);
C_xy_start_obs_rep = repmat(C_xy_start_obs,lenObs,1);

firstImpactTime = inf;
lastImpactTime = inf;

firstImpactFlag = false;

for ii = 1:length(t)
    t_ii=t(ii);
    XY_occ_obs_t_ii=XY_occ_obs+C_xy_start_obs_rep+repmat(t_ii*V_c_obs,lenObs,1);
    XY_occ_obs_t_ii_round=RoundToRes(XY_occ_obs_t_ii,0.02);
    XY_occ_obs_t_ii_round_hash = round(XY_occ_obs_t_ii_round*hash,6);
    
    %     clf
%     figure(1)
%     hold on
%     plotRobotPath(Path)
%     plotRoboticWheelchair(robotPose)
%     plot(XY_occ_obs_t_ii(:,1),XY_occ_obs_t_ii(:,2),'.','Color',co(2,:))
%     axis([-1 3 0.5 1.5])
%     hold off
%     axis equal
%     drawnow
    
    if any(ismember(XY_occ_obs_t_ii_round_hash,PathOccGrid_XY_hash)) && ~ firstImpactFlag
%         pause(1)
        firstImpactFlag=true;
        firstImpactTime = t_ii;
    end
    
    if firstImpactFlag && ~any(ismember(XY_occ_obs_t_ii_round,PathOccGrid_XY,'rows')) 
%         pause(1)
        lastImpactTime = t_ii;
        break
    end
    
end

end
