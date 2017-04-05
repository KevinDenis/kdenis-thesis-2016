function [ MotionPrem ] = Mat2Struct( MotionPremMat,res )
%MAT2STRUCT Summary of this function goes here
%   Detailed explanation goes here
npts=10;
n=length(MotionPremMat);

MotionPrem(1:n,1)=struct('x0',0,'y0',0,'th0',0,...
                               'x1',0,'y1',0,'th1',0,...
                               'X',zeros(npts,1),'Y',zeros(npts,1),'TH',zeros(npts,1),...
                               'kappa',0,'dkappa',0,'Ltot',0,'intKappa',0,...
                               'PathOccGrid',[],'PathOccXY',[],...
                               'pathCost',3*0+0,'free',true);
for ii=1:n
    x0= MotionPremMat(ii,1);
    y0= MotionPremMat(ii,2);
    th0=MotionPremMat(ii,3);
    x1= MotionPremMat(ii,4);
    y1= MotionPremMat(ii,5);
    th1=MotionPremMat(ii,6);
    k = MotionPremMat(ii,7);
    dk= MotionPremMat(ii,8);
    Ltot=MotionPremMat(ii,9);
    intKappa=MotionPremMat(ii,10);
    npts=round(Ltot/res);
    
    s=linspace(0,Ltot,npts);
    ks=k+dk*s;
    [X,Y]=pointsOnClothoid(x0,y0,th0,k,dk,Ltot,npts);
    TH=wrapToPi(cumtrapz(s,ks)+th0);
    TH(1)=th0; TH(end) = th1;
    MotionPrem(ii)=struct('x0',x0,'y0',y0,'th0',th0,...
                                'x1',x1,'y1',y1,'th1',th1,...
                                'X',X.','Y',Y.','TH',TH.',...
                                'kappa',k,'dkappa',dk,'Ltot',Ltot,'intKappa',intKappa, ...
                                'PathOccGrid',[],'PathOccXY',[],...
                                'pathCost',3*Ltot+intKappa,'free',true);
end
                                 
end

