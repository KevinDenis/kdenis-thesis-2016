initWorkspace

XY=rand(1e6,2)*100;

tic
 [ XY_Round ] = RoundToRes_mex(XY,0.02);
 toc
 
 tic
 [ XY_Round ] = RoundToRes(XY,0.02);
 toc