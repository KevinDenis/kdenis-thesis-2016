function plotStateLatticePoints(MotionPrem,LSLset)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

grid_XY=[[MotionPrem.x1].', [MotionPrem.y1].'];
gridSL_XY=zeros(1,2);
nn=1;
for ii=1:length(grid_XY)
x1_ii=grid_XY(ii,1);
y1_ii=grid_XY(ii,2);
if rem(x1_ii,LSLset.dxSL) == 0 && rem(y1_ii,LSLset.dxSL) == 0 && ~(x1_ii ==0 && y1_ii ==0)
    gridSL_XY(nn,:)=grid_XY(ii,:);
    nn=nn+1;
end

end

plot(gridSL_XY(:,1),gridSL_XY(:,2),'o','MarkerEdgeColor','g','MarkerFaceColor','g','MarkerSize',10)
end

