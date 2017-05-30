function [LabGrid,LabGridEdges,XY_occ_lab] = getMapEdgeXYOccFormBmp(bmp,res)
%[LabGrid,LabGridEdges,XY_occ_lab] = getMapEdgeXYOccFormBmp(bmp,res)
LabBW = ~imread(bmp);
LabGrid = robotics.BinaryOccupancyGrid(LabBW,1/res);
LabGrid.GridLocationInWorld=[0 0];
edgesBW = edge(~LabBW,'Roberts');
% imwrite(edgesBW,'RobotLabo_Lift_edges.bmp')
% imwrite(~edgesBW,'RobotLabo_Lift_edges.bmp')
LabGridEdges=robotics.BinaryOccupancyGrid(edgesBW,1/res);
LabGridEdges.GridLocationInWorld=[0 0];

[ii_lab,jj_lab]=meshgrid(1:LabGridEdges.GridSize(1),1:LabGridEdges.GridSize(2));
ii_all_lab=ii_lab(:);
jj_all_lab=jj_lab(:);
occval_lab =getOccupancy(LabGridEdges,[ii_all_lab jj_all_lab], 'grid');
ii_occ_lab=ii_all_lab(occval_lab==1);
jj_occ_lab=jj_all_lab(occval_lab==1);
XY_occ_lab=grid2world(LabGridEdges,[ii_occ_lab jj_occ_lab]);
end

