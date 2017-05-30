function [ grid_XYTH1 ] = getXYTHGridForBezierCurve( grid_XY,LSLset )
%[ grid_XYTH1 ] = getXYTHGridForBezierCurve( grid_XY,LSLset )
MP_cloth=getClothoidFromGrid(grid_XY,LSLset); % speed up the process ...
grid_XYTH1_cloth=[[MP_cloth.x1].',[MP_cloth.y1].',abs([MP_cloth.th1].')];
grid_XYTH1_cloth=unique(grid_XYTH1_cloth,'rows','stable');
[~,idxUnique,~]=unique(grid_XYTH1_cloth(:,1:2),'rows','last'); % get max angle for same XY coordinates
grid_XY_THmax_cloth=grid_XYTH1_cloth(idxUnique,:);

% add all range of -th_max:dth=th_max for each X Y clothoid rachable grid
grid_XYTH1=zeros(1,3);
idx=0;
for ii=1:size(grid_XY_THmax_cloth,1)
    x_ii=grid_XY_THmax_cloth(ii,1);
    y_ii=grid_XY_THmax_cloth(ii,2);
    th_ii=grid_XY_THmax_cloth(ii,3);         
    TH_ii=(-th_ii:LSLset.dth:th_ii).';
    XY_Rep=repmat([x_ii,y_ii],length(TH_ii),1);
    XYTH_ii=[XY_Rep,TH_ii];
    idx=idx(end)+(1:size(XYTH_ii,1));
    grid_XYTH1(idx,:)=XYTH_ii;  
end

end

