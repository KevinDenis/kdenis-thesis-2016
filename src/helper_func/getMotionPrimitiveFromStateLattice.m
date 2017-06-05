function [MP] = getMotionPrimitiveFromStateLattice(LSL)
%[MP] = getMotionPrimitiveFromStateLattice(LSL)
[vertices] = getForwardMotionFromStateLattice(LSL);
idxOrigin=ismember(vertices(:,1:2), [0 0],'rows');
MP=LSL(idxOrigin,:);
end

