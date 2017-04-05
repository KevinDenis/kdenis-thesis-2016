function plotRobotPath(Path)
%PLOTSIMPLEROBOT Summary of this function goes here
%   Detailed explanation goes here

load('RobotHull.mat')
co=get(groot,'DefaultAxesColorOrder');

X_path=[];
Y_path=[];
for ii=1:length(Path)
   X_path=[X_path;Path(ii).X;nan];
   Y_path=[Y_path;Path(ii).Y;nan]; 
end

XY_grid_visited=[[Path.x0].',[Path.y0].';
                 [Path.x1].',[Path.y1].'];

X_robot_rot_trans=[];
Y_robot_rot_trans=[];

for ii=1:length(Path)
    Path_ii=Path(ii);
    for jj=1:5:length(Path_ii.X)
        x_jj=Path_ii.X(jj);
        y_jj=Path_ii.Y(jj);
        th_jj=Path_ii.TH(jj);
        [XY_rot_trans_jj]=RotTransXY(RobotHull,th_jj,x_jj,y_jj);
        X_robot_rot_trans=[X_robot_rot_trans;XY_rot_trans_jj(:,1);nan];
        Y_robot_rot_trans=[Y_robot_rot_trans;XY_rot_trans_jj(:,2);nan];
    end
end

plot(X_path,Y_path,'g','LineWidth',3)
scatter(XY_grid_visited(:,1),XY_grid_visited(:,2),[],'r*')
plot(X_robot_rot_trans,Y_robot_rot_trans,'k','Linewidth',1)
end
