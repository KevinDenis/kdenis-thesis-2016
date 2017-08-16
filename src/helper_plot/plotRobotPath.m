function plotRobotPath(Path,varargin)
%PLOTSIMPLEROBOT Summary of this function goes here
%   Detailed explanation goes here



if nargin == 1
    plotColor = 'g';
    lineWidthPlot=3;
    plotStyle='-';
elseif nargin == 2
    plotColor=(varargin{1});
    lineWidthPlot=3;
    plotStyle='-';
elseif nargin == 3
    plotColor=(varargin{1});
    lineWidthPlot=(varargin{2});
    plotStyle='-';
elseif nargin == 4
    plotColor=(varargin{1});
    lineWidthPlot=(varargin{2});
    plotStyle=(varargin{3});
end



RobotHull=[-0.80  0.20;
           -0.46  0.30;
            0.50  0.30;
            0.50 -0.30;
           -0.46 -0.30;
           -0.80 -0.20;
           -0.80  0.20];
       
for ii=1:length(Path)
    if ~Path(ii).free
        idxPlot=Path(ii).idxBlocked;
        Path(ii).X=Path(ii).X(1:idxPlot-1);
        Path(ii).Y=Path(ii).Y(1:idxPlot-1);
        Path(ii).TH=Path(ii).TH(1:idxPlot-1);
    end
end


ii_todelete=[];
for ii=1:length(Path)
    if isempty(Path(ii).X)
        ii_todelete=[ii_todelete;ii];
    end
end
Path(ii_todelete)=[];

X_path=zeros(1);
Y_path=zeros(1);
kk=0;

for ii=1:length(Path)
    kk=kk(end)+(1:(length(Path(ii).X)+1));
    X_path(kk)=[Path(ii).X;nan];
    Y_path(kk)=[Path(ii).Y;nan];
end

XY_grid_visited=[[Path.x0].',[Path.y0].';
                 [Path.x1].',[Path.y1].'];
             
XY_grid_visited=unique(XY_grid_visited,'rows');

X_robot_rot_trans=zeros(1);
Y_robot_rot_trans=zeros(1);
kk=0;
for ii=1:length(Path)
    Path_ii=Path(ii);
    for jj=1:10:length(Path_ii.X)
        [XY_rot_trans_jj]=RotTransXY(RobotHull,Path_ii.TH(jj),Path_ii.X(jj),Path_ii.Y(jj));
        kk=kk(end)+(1:(length(XY_rot_trans_jj(:,1))+1));
        X_robot_rot_trans(kk)=[XY_rot_trans_jj(:,1);nan];
        Y_robot_rot_trans(kk)=[XY_rot_trans_jj(:,2);nan];
    end
end

plot(X_path,Y_path,'Color',plotColor,'Linewidth',lineWidthPlot);  
% plot(XY_grid_visited(:,1),XY_grid_visited(:,2),'r*')
plot(X_robot_rot_trans,Y_robot_rot_trans,'k','Linewidth',1)
end
