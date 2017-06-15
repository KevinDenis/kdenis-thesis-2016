%==========================================================================
%
%                     Get Local State Lattice Settings
%                     ---------------------------------
% Use disposes of certain parameters to change the Local State Lattice
% Settings :
%   * xmax, ymax = ROI defined by rectagle x0 xmax y0 ymax at orient th0
%   * kmax = maximum curvature of the curve
%   * res = resulution of path, 2x the resolution of the Occupancy Grid
%   * dth = angular discretization
%   * dx1, x1_max = x-y discretization and max width/hight of fine grid
%   * dx2, x2_max = x-y discretization and max width/hight of normal grid
%   * dx3, x3_max, y3_max = x-y discr. and max width/hight of coarse grid
%   * dxEP = parameter defining the Expantion Positions
%
% Kevin DENIS, KU Leuven, 2016-17
% Master Thesis: Path planning algorithm for semi-autonomous
%                mobile robots with fast and accurate collision checking
%
%==========================================================================

%==========================================================================
%
%                 Local State Lattice Structure (LSL)
%                 -----------------------------------
%             [ x0 y0 th0 x1 y1 th1 X Y TH S K k dk Ltot
%              intK PathOccXY pathCost free idxBlocked ID]
%
%                        Obstacle Table Structure
%                        ------------------------
%                     [ x y (path)ID (path)blockedIdx]
%
%==========================================================================


function [LSLset] = getLocalStateLatticeSettings()
res=1e-2; % extra safty factor of 2
LSLset.x0=0;
LSLset.y0=0;
LSLset.th0=0;
LSLset.xmax=2;
LSLset.ymax=1.5;
LSLset.kmax=1;
LSLset.res = res;

LSLset.dth=pi/8;

LSLset.dx1=0.1;
LSLset.x1_max=1;

LSLset.dx2=0.25;
LSLset.x2_max=2;

LSLset.dx3=0.5;
LSLset.x3_max=4;
LSLset.y3_max=3;

LSLset.dxEP=0.5;
LSLset.ROI=[res LSLset.ymax; LSLset.xmax LSLset.ymax; LSLset.xmax -LSLset.ymax;res -LSLset.ymax;res LSLset.ymax];
end