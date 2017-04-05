function plotPath(Path)
%PLOTPATH Summary of this function goes here
%   Detailed explanation goes here
%   Notes :
%   * quickly plots in one shot thanks to nan (could also be inf)
%   * is pre-allocation really quicker in this 
co=get(groot,'DefaultAxesColorOrder');

[Path_k_cte,Path_k_notcte] = sortMotionPremPlot(Path,[],[]);

X_k_cte=[];
Y_k_cte=[];
for ii=1:length(Path_k_cte)
   X_k_cte=[X_k_cte;Path_k_cte(ii).X;nan];
   Y_k_cte=[Y_k_cte;Path_k_cte(ii).Y;nan]; 
end

X_k_notcte=[];
Y_k_notcte=[];
for ii=1:length(Path_k_notcte)
   X_k_notcte=[X_k_notcte;Path_k_notcte(ii).X;nan];
   Y_k_notcte=[Y_k_notcte;Path_k_notcte(ii).Y;nan]; 
end

XY=unique([[Path.x0].',[Path.y0].';[Path.x1].',[Path.y1].'],'rows');
    
plot(X_k_notcte,Y_k_notcte,'Color',co(1,:),'Linewidth',1);  
plot(X_k_cte,Y_k_cte,'Color',co(2,:),'Linewidth',2);
plot(XY(:,1),XY(:,2),'k*')


%% I have to have MotPrem settings to plot this !
%     for ii=1:length(Path)
%         x1_ii=Path(ii).x1;
%         y1_ii=Path(ii).y1;
%         th1_ii=Path(ii).th1;
%         if rem(x1_ii,dx_SL) == 0 && rem(y1_ii,dx_SL) == 0 
%             scatter(x1_ii,y1_ii,300,'g*')
%         end
% 
%     end
end

