function [MotionPrem] = getMotionPremFromGrid(grid_XY,LSLset)
%[MotionPrem] = getMotionPrem(MP_par)
%   Generate a set of Motion Primitives based on  a set of parameters
%   First, a grid is made based on the MP_par.
%   Then, paths are 
npts=1;

x0=LSLset.x0;
xmax=LSLset.xmax;
y0=LSLset.y0;
ymax=LSLset.ymax;
th0=LSLset.th0;
dth=LSLset.dth;
kappa_max=LSLset.kmax;
res=LSLset.res;

TH=wrap2Pi(-pi/2:dth:pi/2);
TH_rot=wrap2Pi(TH+th0);


ROI_Rot_Trans=RotTransXY(LSLset.ROI,th0,x0,y0);
idxInROI=inpolygon(grid_XY(:,1),grid_XY(:,2),ROI_Rot_Trans(:,1),ROI_Rot_Trans(:,2));
grid_XY_ROI=grid_XY(idxInROI,:);

grid_XYTH1=addTHtoGridXY(grid_XY_ROI,TH_rot);
dX=grid_XYTH1(:,1)-x0;
dY=grid_XYTH1(:,2)-y0;

grid_XYTH1=grid_XYTH1(abs(dX)+abs(dY)>0,:);

MotionPrem(1:2,1)=struct('x0',0,'y0',0,'th0',0,...
                         'x1',0,'y1',0,'th1',0,...
                         'X',zeros(npts,1),'Y',zeros(npts,1),'TH',zeros(npts,1),...
                         'k',0,'dk',0,'Ltot',0,'intK',0,...
                         'PathOccXY',[], 'pathCost',[],'free',true,'ID',[]);
nn=1;
for ii=1:length(grid_XYTH1)
    x1=grid_XYTH1(ii,1); 
    y1=grid_XYTH1(ii,2); 
    th1=grid_XYTH1(ii,3);
    [k,dk,Ltot,~] = buildClothoid(x0,y0,th0,x1,y1,th1);
    if abs(k) <= kappa_max && abs(k+dk*Ltot) <= kappa_max
        npts=round(Ltot/res);
        s=linspace(0,Ltot,npts);
        ks=k+dk*s;
        intK=trapz(s,abs(ks));
        [X,Y]=pointsOnClothoid(x0,y0,th0,k,dk,Ltot,npts);
        TH=wrapToPi(cumtrapz(s,ks)+th0);
        TH(1)=th0; 
        TH(end) = th1;
        MotionPrem(nn).x0 = x0;
        MotionPrem(nn).y0 = y0;
        MotionPrem(nn).th0= th0;
        MotionPrem(nn).x1 = x1;
        MotionPrem(nn).y1 = y1;
        MotionPrem(nn).th1= th1;
        MotionPrem(nn).X  = X.';
        MotionPrem(nn).Y  = Y.';           
        MotionPrem(nn).TH = TH.';
        MotionPrem(nn).k  = k;
        MotionPrem(nn).dk = dk;
        MotionPrem(nn).Ltot=Ltot;
        MotionPrem(nn).intK=intK;
        MotionPrem(nn).pathCost=3*Ltot+intK;
        MotionPrem(nn).free=true;
        nn=nn+1;
    end
end
end

function [grid_X_Y_TH_rot_trans] = GridRotTrans(MP_par)
%[grid_X_Y_rot_trans] = GridRotTrans(grid_X_Y,th,x,y,dx)
%   Detailed explanation goes here
%   notes :
%   * TH is not checked for discritisation -> this should be OK !

x0=MP_par.x0;
y0=MP_par.y0;
th0=MP_par.th0;
dx=MP_par.dx;
x_max=MP_par.x_max;
y_max=MP_par.y_max;
dth=MP_par.dth;

X=0:MP_par.dx:x_max; 
Y=-y_max:MP_par.dx:y_max;
TH=wrap2Pi(-pi/2:dth:pi/2);

grid_X_Y=getAllComb(X,Y); % only OK in this case !
[X_rot,Y_rot]=Rotate0Theta(grid_X_Y(:,1),grid_X_Y(:,2),th0);
X_rot_trans=X_rot+x0;
Y_rot_trans=Y_rot+y0;
TH_rot=wrap2Pi(TH+th0);

X_rot_trans = interp1(X_rot_trans,1:0.25:numel(X_rot_trans)).';
Y_rot_trans = interp1(Y_rot_trans,1:0.25:numel(Y_rot_trans)).';

X_rot_trans_disc=round(X_rot_trans/dx)*dx;
Y_rot_trans_disc=round(Y_rot_trans/dx)*dx;
grid_X_Y_rot_trans=[X_rot_trans_disc,Y_rot_trans_disc];

grid_X_Y_rot_trans =  unique(grid_X_Y_rot_trans,'rows');
grid_X_Y_TH_rot_trans= addTHtoGridXY(grid_X_Y_rot_trans,TH_rot);
grid_X_Y_TH_rot_trans = unique(grid_X_Y_TH_rot_trans,'rows'); 
end

function grid_X_Y_TH=addTHtoGridXY(grid_X_Y,TH)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
grid_X_Y_rep=repmat(grid_X_Y,length(TH),1);
TH_rep=repelem(TH,length(grid_X_Y)).';
grid_X_Y_TH=[grid_X_Y_rep TH_rep];
grid_X_Y_TH=unique(grid_X_Y_TH,'rows');
end