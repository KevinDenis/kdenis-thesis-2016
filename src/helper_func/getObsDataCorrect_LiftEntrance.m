% initWorkspace

[labgrid,obs_out] = getXYOccFormBmpMap('LiftCorners.bmp',0.02);
% show(labgrid)




% obs_x=obs_out(:,1);
% obs_y=obs_out(:,2);
% figure()
% scatter(obs_x,obs_y,'k','filled')
% axis equal
% hold on
% idxGood=zeros(length(obs_out),1);
% for kk=1:length(obs_out)
%     [x,y]=ginput(1);
%     dr = kron(ones(length(obs_out),1),[x y]) - obs_out;
%     [min_val, min_id] = min(sum(dr.^2,2));
%     scatter(obs_x(min_id),obs_y(min_id),'b','filled')
%     idxGood(kk)=min_id;
% end

obs_out=obs_out(idxGood,:)
plot(obs_out(:,1),obs_out(:,2),'k')
