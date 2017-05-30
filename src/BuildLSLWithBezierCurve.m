%=========================================================================%
%                                                                         %
%          Build Local State Lattice With Grid using Bézier Curve         %
%          -------------------------------------------------------        %
%                                                                         %
% Overview :                                                              %
%   * Discrete end poses defined by the multi size grid are connected with% 
%   a Bézier Curve by solving a Constrainted Optimization Problem (COP)   % 
%   using Casadi (see BezierCOP for more information)                     %
%   * since this process is very slow and the Bézier Curve is not the     %
%   selected curve to be the base of the LSL, only the first step of the  %
%   creation of the LSL has been calculated, beeing the MP starting from  %
%   the origin.                                                           %
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

function [LSL,MP]=BuildLSLWithBezierCurve(grid_XY,LSLset)

%% Motion Primitive from [0 0 0]
[MP] = getBezierCurveFromGrid(grid_XY,LSLset);
% LSLset_mod=LSLset;

%% State Lattice Set by generating Motion Primitive at State Lattice Positions
% VERY SLOW
LSL=MP;
% n=length(MP);
% idxLSL=n;
% for ii=1:n
%     x1_ii=MP(ii).x1;
%     y1_ii=MP(ii).y1;
%     th1_ii=MP(ii).th1;
%     if rem(x1_ii,LSLset.dxSL) == 0 && rem(y1_ii,LSLset.dxSL) == 0
%         LSLset_mod.x0=x1_ii;
%         LSLset_mod.y0=y1_ii;
%         LSLset_mod.th0=th1_ii;
%         [MP_EP] = getBezierCurveFromGrid(grid_XY,LSLset_mod);
%         idxLSL=idxLSL(end)+(1:length(MP_EP));
%         LSL(idxLSL)=MP_EP;
%     end
% end
% 
% LSL = AddReverseDirection(LSL);
% LSL = [getMotionPremTurnOnSpot(LSL,0);LSL];
% LSL = CleanupLSL(LSL);
% LSL = FreeAllPaths(LSL);
end