initWorkspace

defaultStartEnd=true;

LSLset=getLocalStateLatticeSettings();
LSLset.xmax=2;
LSLset.ymax=1;
LSLset.dth = pi/4;
LSLset.dx1=0.25;
LSLset.x1_max=0.5;
LSLset.dx2=0.25;
LSLset.x2_max=1;
LSLset.dx3=0.25;
LSLset.x3_max=2;
LSLset.y3_max=2;
res=LSLset.res;
LSLset.ROI=[res LSLset.ymax; LSLset.xmax LSLset.ymax; LSLset.xmax -LSLset.ymax;res -LSLset.ymax;res LSLset.ymax];


LSL=DMP_BuildLSLForDMP(LSLset);
LSL=DMP_BuildOccGridForDMP(LSL);
DMP_SLFreePath_Voronoi(LSL,defaultStartEnd)