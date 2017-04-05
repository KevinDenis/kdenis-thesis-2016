clc
close all


ObstacleTable(1:2,1)=struct('xy',[],'paths',[]);



% 
% progressbar('Obstacle')
% n=length(ObstacleTable);


% prep all visited grids
XY_all=zeros(1,2);
kk=0;
m=length(StateLattice);
% progressbar('Obstacle') 
for ii=1:m
    kk=kk(end)+[1:size([StateLattice(ii).PathOccXY],1)];
    XY_all(kk,:)=[StateLattice(ii).PathOccXY];
%     progressbar(ii/m)
end
progressbar('Obstacle') 
XY_all=unique(XY_all,'rows');
n=length(XY_all);
for ii=1:n
   xy_ii=XY_all(ii,:);
   ObstacleTable(ii).xy=xy_ii;
   for jj=1:m
       PathOccXY_jj=[StateLattice(jj).PathOccXY];
       if ~isnan(findrow_mex(PathOccXY_jj,xy_ii))
           ObstacleTable(ii).paths=[ObstacleTable(ii).paths,StateLattice(jj).ID];
       end
   end
   progressbar(ii/n)
end