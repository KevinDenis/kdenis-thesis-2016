function plotReachableEndPoses(Path,robotPose)
%plotReachableEndPoses(Path,robotPose)
%   Detailed explanation goes here

PathFree=Path([Path.free]);

vertices = getStartEndVerticesPath(PathFree);
vertices_end = unique(vertices(:,4:6),'rows');
X_endPose=zeros(size(vertices_end,1),3);
Y_endPose=X_endPose;

Linf = max(abs(repmat(robotPose(:,1:2),size(vertices_end(:,1:2),1),1)-vertices_end(:,1:2)),[],2);
idxGrid1=Linf<=1;

r=0.05;
X_endPose(idxGrid1,:)=[vertices_end(idxGrid1,1), vertices_end(idxGrid1,1)+r*cos(vertices_end(idxGrid1,3)), nan(size(vertices_end(idxGrid1,1)))];
Y_endPose(idxGrid1,:)=[vertices_end(idxGrid1,2), vertices_end(idxGrid1,2)+r*sin(vertices_end(idxGrid1,3)), nan(size(vertices_end(idxGrid1,1)))];

idxGrid2=Linf>1 & Linf<=2;
r=0.1;
X_endPose(idxGrid2,:)=[vertices_end(idxGrid2,1), vertices_end(idxGrid2,1)+r*cos(vertices_end(idxGrid2,3)), nan(size(vertices_end(idxGrid2,1)))];
Y_endPose(idxGrid2,:)=[vertices_end(idxGrid2,2), vertices_end(idxGrid2,2)+r*sin(vertices_end(idxGrid2,3)), nan(size(vertices_end(idxGrid2,1)))];

idxGrid3=Linf>2;
r=0.1;
X_endPose(idxGrid3,:)=[vertices_end(idxGrid3,1), vertices_end(idxGrid3,1)+r*cos(vertices_end(idxGrid3,3)), nan(size(vertices_end(idxGrid3,1)))];
Y_endPose(idxGrid3,:)=[vertices_end(idxGrid3,2), vertices_end(idxGrid3,2)+r*sin(vertices_end(idxGrid3,3)), nan(size(vertices_end(idxGrid3,1)))];

X_endPose=X_endPose.';
Y_endPose=Y_endPose.';

plot(X_endPose(:).',Y_endPose(:).','k','LineWidth',1.5)

% XY=unique([[PathFree.x0].',[PathFree.y0].';[PathFree.x1].',[PathFree.y1].'],'rows');
% if ~isempty(XY) && size(XY,1)>5; plot(XY(:,1),XY(:,2),'k*','MarkerSize',15,'Linewidth',1.1); end

end

