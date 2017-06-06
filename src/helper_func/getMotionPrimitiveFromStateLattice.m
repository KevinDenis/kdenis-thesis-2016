function [MP] = getMotionPrimitiveFromStateLattice(LSL)
%[MP] = getMotionPrimitiveFromStateLattice(LSL)
[vertices] = getStartEndVerticesPath(LSL);
idxOrigin=ismember(vertices(:,1:2), [0 0],'rows');
MP=LSL(idxOrigin,:);
end

