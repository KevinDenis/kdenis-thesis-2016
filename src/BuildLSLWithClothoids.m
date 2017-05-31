%=========================================================================%
%                                                                         %
%             Build Local State Lattice With Grid using Clothoids         %
%             ---------------------------------------------------         %
%                                                                         %
% Overview :                                                              %
%   * Discrete end poses defined by the multi-size grid are connected with%
% a clothoid. If this clothoid is compliant with the constraints, it is   %
% added to the Local State Lattice Structure containing all feasible paths%
% starting from the robot actual position0.                               %
%   * First, every CEP defined by the ROI at the origin [0 0 0°]          %
%   * Then this step is repeated at every feasible Expansion Position (EP)%
% defined by dxEP. At these discrete poses, the first step is repeated,   %
% connecting feasible paths with this EP, creating a larger variety of    %
% paths.                                                                  %
% * Paths within the local state lattice set are therefore defined        %
% by 1 or 2 clothoids.                                                    %
%                                                                         %
%                           !!! IMPORTANT !!!                             %
%                           -----------------                             %
% Code to generate clothoids is not written by Kevin DENIS, but originates%
% from : https://github.com/ebertolazzi/G1fitting                         %
% mex files for fast computations are pre-compiled for Linux and Windows  %
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

function [LSL,MP]=BuildLSLWithClothoids(grid_XY,LSLset)
%% Motion Primitive from [0 0 0]
[MP] = getClothoidFromGrid(grid_XY,LSLset);
LSLset_mod=LSLset; % modified LSLset for later use for Expantion Positions

%% State Lattice Set by generating Motion Primitive at State Lattice Positions
LSL=MP;
n=length(MP);
idxSL=n;
for ii=1:n
    x1_ii=MP(ii).x1;
    y1_ii=MP(ii).y1;
    th1_ii=MP(ii).th1;
    if rem(x1_ii,LSLset.dxEP) == 0 && rem(y1_ii,LSLset.dxEP) == 0 % feasable end pose is an Expantion Position
        LSLset_mod.x0=x1_ii;
        LSLset_mod.y0=y1_ii;
        LSLset_mod.th0=th1_ii;
        [MP_EP] = getClothoidFromGrid(grid_XY,LSLset_mod);
        idxSL=idxSL(end)+(1:length(MP_EP));
        LSL(idxSL)=MP_EP;
    end
end
LSL = AddReverseDirection(LSL);
LSL = [getMotionPremTurnOnSpot(LSL,0);LSL];
LSL = CleanupLSL(LSL);
LSL = FreeAllPaths(LSL);
end