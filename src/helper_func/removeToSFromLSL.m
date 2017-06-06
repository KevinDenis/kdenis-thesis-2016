function LSL = removeToSFromLSL(LSL)
%LSL = removeToSFromLSL(LSL)
%   Detailed explanation goes here
vertices = getStartEndVerticesPath(LSL);
vertices_XY_0 = vertices(:,1:2);
vertices_XY_1 = vertices(:,4:5);
toDelete = all(vertices_XY_0 == vertices_XY_1,2);
LSL(toDelete)=[];
end

