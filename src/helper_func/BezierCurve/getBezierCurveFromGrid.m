function [MP] = getBezierCurveFromGrid(grid_XY,LSLset)
%[MP] = getBezierCurveFromGrid(grid_XY,LSLset)
%   Connect Candidate End Poses (CEP) ( discrete grid_XY_TH withing the    
%   Regoin of Interest (ROI) ) with Bézier Curves. 
%   COP is solved for each CEP. If it is feasable, it is added to the set 
%   of Motion Primitives.
%   See BezierCOP for more information
%
%    Kevin DENIS, KU Leuven, 2016-17
%    Master Thesis: Path planning algorithm for semi-autonomous
%                   mobile robots with fast and accurate collision checking

% Here, all nececery information for selecting the candidate end poses,
% which are the the grid cells withing the Region of Interest are retrieved
% from the Local State Lattice Setting structure
npts=1;
n=4;
t=linspace(0,1,101)'; % overall resolution of the solver
x0=LSLset.x0;
y0=LSLset.y0;
th0=LSLset.th0;
dth=LSLset.dth;
kappa_max=LSLset.kmax;
ROI=LSLset.ROI;
TH=wrap2Pi(-pi/2:dth:pi/2);
TH_rot=wrap2Pi(TH+th0);

ROI_Rot_Trans=RotTransXY(ROI,th0,x0,y0);
idxInROI=InPolygon(grid_XY(:,1),grid_XY(:,2),ROI_Rot_Trans(:,1),ROI_Rot_Trans(:,2));
grid_XY_ROI=grid_XY(idxInROI,:);

grid_XYTH1=addTHtoGridXY(grid_XY_ROI,TH_rot);

grid_XYTH1=grid_XYTH1(grid_XYTH1(:,1)>=0.1,:);

MP(1:2,1)=struct('x0',0,'y0',0,'th0',0,...
                 'x1',0,'y1',0,'th1',0,...
                 'X',zeros(npts,1),'Y',zeros(npts,1),'TH',zeros(npts,1),...
                 'S',zeros(npts,1),'K',zeros(npts,1), ... 
                 'k',0,'dk',0,'Ltot',0,'intK',0,...
                 'PathOccXY',[], 'pathCost',[], ...
                 'free',true,'idxBlocked',[],'ID',[]);
nn=1; % needed, since one can't know in advance how many MP can be drawn
progressbar('Bézier Curve')
for ii=1:size(grid_XYTH1,1)
    x1=grid_XYTH1(ii,1); 
    y1=grid_XYTH1(ii,2); 
    th1=grid_XYTH1(ii,3);
    P0=[x0 y0 th0]; % start pose
    P1=[x1 y1 th1]; % end pose
    initCOP=struct('P0',P0,'P1',P1,'n',n,'t',t,'kappa_max',kappa_max); % COP solution. Empty if not feasible
    optPath=BezierCOP(initCOP);
    if ~isempty(optPath)
        MP(nn) = optPath; % add feasible Bézier Curve to set of Curves
        nn=nn+1;
    end
    progressbar(ii/size(grid_XYTH1,1))
end
end

function grid_X_Y_TH=addTHtoGridXY(grid_X_Y,TH)
%grid_X_Y_TH=addTHtoGridXY(grid_X_Y,TH) Summary of this function goes here
%   Detailed explanation goes here
grid_X_Y_rep=repmat(grid_X_Y,length(TH),1);
TH_rep=repelem(TH,length(grid_X_Y)).';
grid_X_Y_TH=[grid_X_Y_rep TH_rep];
grid_X_Y_TH=unique(grid_X_Y_TH,'rows');
end