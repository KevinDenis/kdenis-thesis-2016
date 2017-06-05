function [MP] = getClothoidFromGrid(grid_XY,LSLset)
%[MP] = getClothoidFromGrid(grid_XY,LSLset)
%   Connect Candidate End Poses (CEP) ( discrete grid_XY_TH withing the    
%   Regoin of Interest (ROI) ) with cloithoids. 
%   If the clothoid complies with the constraints, it is added to the set 
%   of Motion Primitives.
%   This is done at the origin (Robot Pose = [ 0 0 0°], but also at the
%   Expantion Positions (EP). For EP, the LSLset is therefore changed to
%   contain Robot Pose = [x0 y0 th0]. EP are a subset feasable of CEP
%   defined by LSLset.dxEP.
%
%    Kevin DENIS, KU Leuven, 2016-17
%    Master Thesis: Path planning algorithm for semi-autonomous
%                   mobile robots with fast and accurate collision checking

% Here, all nececery information for selecting the candidate end poses,
% which are the the grid cells withing the Region of Interest are retrieved
% from the Local State Lattice Setting structure
npts=1;
x0=LSLset.x0;
y0=LSLset.y0;
th0=LSLset.th0;
dth=LSLset.dth;
kappa_max=LSLset.kmax;
res=LSLset.res;
ROI=LSLset.ROI;
TH=wrap2Pi(-pi/2:dth:pi/2);
TH_rot=wrap2Pi(TH+th0);


ROI_Rot_Trans=RotTransXY(ROI,th0,x0,y0);
idxInROI=InPolygon(grid_XY(:,1),grid_XY(:,2),ROI_Rot_Trans(:,1),ROI_Rot_Trans(:,2));
grid_XY_ROI=grid_XY(idxInROI,:);

grid_XYTH1=addTHtoGridXY(grid_XY_ROI,TH_rot);

% Initializes the structure for the Motion Primitives (same as LSL)
MP(1:2,1)=struct('x0',0,'y0',0,'th0',0,...
                         'x1',0,'y1',0,'th1',0,...
                         'X',zeros(npts,1),'Y',zeros(npts,1),'TH',zeros(npts,1),...
                         'S',zeros(npts,1),'K',zeros(npts,1), ... 
                         'k',0,'dk',0,'Ltot',0,'intK',0,...
                         'PathOccXY',[], 'pathCost',[], ...
                          'free',true,'idxBlocked',[],'ID',[]);
nn=1; % needed, since one can't know in advance how many MP can be drawn to each Candidate End Pose
for ii=1:length(grid_XYTH1)
    x1=grid_XYTH1(ii,1); 
    y1=grid_XYTH1(ii,2); 
    th1=grid_XYTH1(ii,3);
    [k,dk,Ltot,~] = buildClothoid(x0,y0,th0,x1,y1,th1);
    if abs(k) <= kappa_max && abs(k+dk*Ltot) <= kappa_max % if curve is complient with constraints, add it to the set of Motion Primitivess
        S=0:res:Ltot;
        [X,Y]=pointsOnClothoid(x0,y0,th0,k,dk,Ltot,length(S));
        TH = th0 + S.*(k+S*(dk/2));
        K=k+S*dk ; 
        intK=trapz(S,abs(K));
        MP(nn).x0 = x0;
        MP(nn).y0 = y0;
        MP(nn).th0= th0;
        MP(nn).x1 = x1;
        MP(nn).y1 = y1;
        MP(nn).th1= th1;
        MP(nn).X  = X.';
        MP(nn).Y  = Y.';           
        MP(nn).TH = TH.';
        MP(nn).S = S.';
        MP(nn).K = K.';
        MP(nn).k  = k;
        MP(nn).dk = dk;
        MP(nn).Ltot=Ltot;
        MP(nn).intK=intK;
        MP(nn).pathCost=3*Ltot+intK;
        MP(nn).free=true;
        nn=nn+1;
    end
end
end

function grid_X_Y_TH=addTHtoGridXY(grid_X_Y,TH)
%grid_X_Y_TH=addTHtoGridXY(grid_X_Y,TH)
%   Adds end positions pased on TH to grid_X_Y
grid_X_Y_rep=repmat(grid_X_Y,length(TH),1);
TH_rep=repelem(TH,length(grid_X_Y)).';
grid_X_Y_TH=[grid_X_Y_rep TH_rep];
grid_X_Y_TH=unique(grid_X_Y_TH,'rows');
end