function plotCircle(pose,varargin)
co=get(groot,'DefaultAxesColorOrder');

%plotRoboticWheelchair(pose)
%   The footprint of the wheelchair is plotted along with it's reference
%   frame R.

switch nargin
    case 1
        plotColor = co(3,:);
        setLineWidth = 3;
    case 2
        plotColor = (varargin{1});
        setLineWidth = 3;
    case 3
        plotColor = (varargin{1});
        setLineWidth = (varargin{2});
end
        
        

% Wheelchair footprint coordinates
RobotHull=[
   -0.0303    0.1403
   -0.0603    0.1303
   -0.0903    0.1103
   -0.1003    0.1003
   -0.1103    0.0903
   -0.1303    0.0603
   -0.1403    0.0303
   -0.1403   -0.0297
   -0.1303   -0.0597
   -0.1203   -0.0797
   -0.1103   -0.0897
   -0.0803   -0.1197
   -0.0603   -0.1297
   -0.0303   -0.1397
    0.0297   -0.1397
    0.0597   -0.1297
    0.0797   -0.1197
    0.0897   -0.1097
    0.1197   -0.0797
    0.1297   -0.0597
    0.1397   -0.0297
    0.1397    0.0303
    0.1297    0.0603
    0.1197    0.0803
    0.0797    0.1203
    0.0597    0.1303
    0.0297    0.1403
   -0.0303    0.1403];      
RobotHull=RotTransXY(RobotHull,pose(3),pose(1),pose(2));

% Reference frame parameters
L=0.1;
W=0.05;
th=30*pi/180;
R_X_arrow =[ 0          , 0
             L          , 0
             L-W*cos(th), W*sin(th);
             nan        , nan;
             L          , 0;
             L-W*cos(th), -W*sin(th)]; 
         
R_X_frame = RotTransXY(R_X_arrow,pose(3),pose(1),pose(2));      
% R_Y_frame = RotTransXY(R_X_arrow,pose(3)+pi/2,pose(1),pose(2));
% R_XY_frame=[R_X_frame;nan,nan;R_Y_frame];

RobotHull_Frame=[RobotHull;nan,nan;R_X_frame];

plot(RobotHull_Frame(:,1),RobotHull_Frame(:,2),'Color',plotColor,'Linewidth',setLineWidth)
end