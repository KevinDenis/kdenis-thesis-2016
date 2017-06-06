%=========================================================================%
%                                                                         %
%              Build Local State Lattice With Grid using circles          %
%              -------------------------------------------------          %
%                                                                         %
% Overview :                                                              %
%   * Discrete set of paths are calculated by integrating achievable      %
%   velocities defined by v,w paires. This is a "forward" approach of     %
%   creating the LSL. Bézier and Clothoid LSL building uses the inverse   %
%   approach (first discrete end poses are defined, then feasible         %
%   trajectories are calculated. Trajactories are automatically feasible  %
%   when integrating feasible velocities                                  %
%   * LSLset.res determines the integration step (resolution of the path) %
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

function [LSL]=BuildLSLWithCircles(LSLset)
res=LSLset.res;
n=500; % there are 200 paths in total, forward and backward.
th=linspace(0,2*pi,n+1); th(end)=[];
v=sin(th);
w=cos(th);
v=v(v>1e-3);
w=w(v>1e-3);
k=w./v;

idxKeep=abs(k)<=1;
k=k(idxKeep);
v=v(idxKeep);
w=w(idxKeep);

clearvars StateLattice
                      
LSL(1:length(v),1)=struct('x0',0,'y0',0,'th0',0,...
                         'x1',[],'y1',[],'th1',[],...
                         'X',[],'Y',[],'TH',[],...
                         'S',[],'K',[], ... 
                         'k',0,'dk',0,'Ltot',0,'intK',0,...
                         'PathOccXY',[], 'pathCost',[], ...
                          'free',true,'idxBlocked',[],'ID',[]);
                      
for ii=1:length(v)
    v_ii=v(ii);
    w_ii=w(ii);
    dt=res/v_ii;
    t=0:dt:4;
    m=length(t);
    X=zeros(m,1);
    Y=zeros(m,1);
    TH=zeros(m,1);
    S=zeros(m,1);
    K=repelem(k(ii),m,1);
    for jj=2:length(t)
        dx=cos(TH(jj-1))*v_ii*dt;
        dy=sin(TH(jj-1))*v_ii*dt;
        dth=w_ii*dt;
        X(jj)=X(jj-1)+dx;
        Y(jj)=Y(jj-1)+dy;
        TH(jj)= TH(jj-1)+dth;
        S(jj)=S(jj-1)+sqrt(dx^2+dy^2);
    end
    Ltot=S(end);
    intK=trapz(S,abs(K));
    LSL(ii).x0 = 0;
    LSL(ii).y0 = 0;
    LSL(ii).th0= 0;
    LSL(ii).x1 = X(end);
    LSL(ii).y1 = Y(end);
    LSL(ii).th1= TH(end);
    LSL(ii).X  = X;
    LSL(ii).Y  = Y;           
    LSL(ii).TH = TH;
    LSL(ii).S = S;
    LSL(ii).K = K;
    LSL(ii).k  = k(ii);
    LSL(ii).dk = 0;
    LSL(ii).Ltot=Ltot;
    LSL(ii).intK=intK;
    LSL(ii).pathCost=3*Ltot+intK;
    LSL(ii).free=true;
end
LSL =  AddReverseDirection(LSL);
LSL = [getMotionPremTurnOnSpot(LSL,0);LSL];
LSL = CleanupLSL(LSL);
LSL = FreeAllPaths(LSL);
end