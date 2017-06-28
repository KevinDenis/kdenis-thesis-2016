function [LabGrid,XY_occ_lab] = getMapXYOccFormBmp(bmp,res)
%[LabGrid,LabGridEdges,XY_occ_lab] = getMapEdgeXYOccFormBmp(bmp,res)
LabBW = ~imread(bmp);
LabGrid = robotics.BinaryOccupancyGrid(LabBW,1/res);
LabGrid.GridLocationInWorld=[0 0];
[ii_lab,jj_lab]=meshgrid(1:LabGrid.GridSize(1),1:LabGrid.GridSize(2));
ii_all_lab=ii_lab(:);
jj_all_lab=jj_lab(:);
occval_lab =getOccupancy(LabGrid,[ii_all_lab jj_all_lab], 'grid');
ii_occ_lab=ii_all_lab(occval_lab==1);
jj_occ_lab=jj_all_lab(occval_lab==1);
XY_occ_lab=grid2world(LabGrid,[ii_occ_lab jj_occ_lab]);
end