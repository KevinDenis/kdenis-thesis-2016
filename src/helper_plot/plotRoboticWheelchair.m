function plotRoboticWheelchair(pose)
%plotRoboticWheelchair(pose)
%   The footprint of the wheelchair is plotted along with it's reference
%   frame R.

% Wheelchair footprint coordinates
RobotHull=[-0.80  0.20;
           -0.46  0.30;
            0.50  0.30;
            0.50 -0.30;
           -0.46 -0.30;
           -0.80 -0.20;
           -0.80  0.20];      
RobotHull=RotTransXY(RobotHull,pose(3),pose(1),pose(2));

% Reference frame parameters
L=0.2;
W=0.05;
th=30*pi/180;
R_X_arrow =[ 0          , 0
             L          , 0
             L-W*cos(th), W*sin(th);
             nan        , nan;
             L          , 0;
             L-W*cos(th), -W*sin(th)]; 
         
R_X_frame = RotTransXY(R_X_arrow,pose(3),pose(1),pose(2));      
R_Y_frame = RotTransXY(R_X_arrow,pose(3)+pi/2,pose(1),pose(2));
R_XY_frame=[R_X_frame;nan,nan;R_Y_frame];

RobotHull_Frame=[RobotHull;nan,nan;R_XY_frame];

plot(RobotHull_Frame(:,1),RobotHull_Frame(:,2),'k','Linewidth',3)
end
