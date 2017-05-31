%=========================================================================%
%                                                                         %
%                           Build Multi Grid                              %
%                           ----------------                              %
%                                                                         %
% Creates a multi-size grid based on LSLsettings (LSLset structure)       %
%                                                                         %
% Overview :                                                              %
%   * Builds a multi-size grid based on the LSLset structure              %
%   * 3 grid sizes are used, fine close to the origin and coarse far away %
%   * A end pose in the coarse grid represents thus a larger group of     %
% possible end poses compared to the fine grid. This emphasizes the fact  %
% that it is not useful to keep a large set of (relatively) similar paths %
% with a end pose that is far from the origin.                            %
%   * each grid size within the Region of Interest (see next step) will be%
% connected with the selected curve geometry. If this curve is feasible, %
% it will be added to the set of paths of the Local State Lattice.        %
%                                                                         %
% grid 1 : 0.10x0.10 --> 1.0m x 1.0m  (Fine grid)                         %
% grid 2 : 0.25x0.25 --> 2.0m x 2.0m  (Medium grid)                       %
% grid 3 : 0.50x0.50 --> 6.0m x 6.0m  (Coarse grid)                       %
%                                                                         %
% Kevin DENIS, KU Leuven, 2016-17                                         %
% Master Thesis: Path planning algorithm for semi-autonomous              %
%                mobile robots with fast and accurate collision checking  %
%                                                                         %
%=========================================================================%


%=========================================================================%
%                                                                         %
%                 Local State Lattice Structure (LSL)                     %
%                 -----------------------------------                     %
%             [ x0 y0 th0 x1 y1 th1 X Y TH S K k dk Ltot                  %
%              intK PathOccXY pathCost free idxBlocked ID]                %
%                                                                         %
%                        Obstacle Table Structure                         %
%                        ------------------------                         %
%                     [ x y (path)ID (path)blockedIdx]                    %
%                                                                         %
%=========================================================================%

function [grid_XY,ROI0,idxIn0,ROI1,idxIn1]=BuildMultiSizeGrid(LSLset)

%% Grid 1
dx1=LSLset.dx1;% dx1 = 0.25;
x1_max=LSLset.x1_max;% x1_max=0;
X1=0:dx1:x1_max;
Y1=X1;
grid_XY1=getAllComb(X1,Y1);

%% Grid 2
dx2=LSLset.dx2;% dx2=0.5;
x2_max=LSLset.x2_max;% x2_max=0;
X2=0:dx2:x2_max;
Y2=X2;
grid_XY2=getAllComb(X2,Y2);
grid_XY2_Linf=max(grid_XY2,[],2);
grid_XY2(grid_XY2_Linf<=x1_max,:)=[];

%% Grid 3
dx3=LSLset.dx3;
x3_max=LSLset.x3_max;
y3_max=LSLset.y3_max;
X3=0:dx3:x3_max;
Y3=0:dx3:y3_max;
grid_XY3=getAllComb(X3,Y3);
grid_XY3_Linf=max(grid_XY3,[],2);
grid_XY3(grid_XY3_Linf<=x2_max,:)=[];

%% Full Grid
grid_XY=[grid_XY1;grid_XY2;grid_XY3];
grid_XY=unique(grid_XY,'rows');
grid_XY=[grid_XY(:,1) grid_XY(:,2); grid_XY(:,1) -grid_XY(:,2)];
grid_XY=unique(grid_XY,'rows');
grid_XY=[grid_XY(:,1) grid_XY(:,2); -grid_XY(:,1) grid_XY(:,2)];
grid_XY=unique(grid_XY,'rows');

ROI0=LSLset.ROI;
idxIn0=InPolygon(grid_XY(:,1),grid_XY(:,2),ROI0(:,1),ROI0(:,2));

ROI1=RotTransXY(ROI0,1.1781,1.5,1);
idxIn1=InPolygon(grid_XY(:,1),grid_XY(:,2),ROI1(:,1),ROI1(:,2));
end