function plotPath(Path)
%PLOTPATH Summary of this function goes here
%   Detailed explanation goes here
%   Notes :
%   * quickly plots in one shot thanks to nan (could also be inf)
co=get(groot,'DefaultAxesColorOrder');

[Path_Circ,Path_Cloth] = sortPathPlot(Path);

X_Circ=zeros(1);
Y_Circ=zeros(1);
kk=0;
for ii=1:length(Path_Circ)
    kk=kk(end)+(1:(length(Path_Circ(ii).X)+1));
    X_Circ(kk)=[Path_Circ(ii).X;nan];
    Y_Circ(kk)=[Path_Circ(ii).Y;nan]; 
end


X_Cloth=zeros(1);
Y_Cloth=zeros(1);
kk=0;
for ii=1:length(Path_Cloth)
    kk=kk(end)+(1:(length(Path_Cloth(ii).X)+1));
    X_Cloth(kk)=[Path_Cloth(ii).X;nan];
    Y_Cloth(kk)=[Path_Cloth(ii).Y;nan]; 
end

XY=unique([[Path.x0].',[Path.y0].';[Path.x1].',[Path.y1].'],'rows');
    
plot(X_Cloth,Y_Cloth,'Color',co(1,:),'Linewidth',1);  
plot(X_Circ,Y_Circ,'Color',co(2,:),'Linewidth',2);
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

