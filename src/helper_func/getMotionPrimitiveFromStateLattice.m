function [MP] = getMotionPrimitiveFromStateLattice(LSL)
%[MP] = getMotionPrimitiveFromStateLattice(LSL)
vertices=[[LSL.x0].',[LSL.y0].',abs([LSL.th0].'), ...
       [LSL.x1].',[LSL.y1].',abs([LSL.th1].')];
idxOrigin=ismember(vertices(:,1:3), [0 0 0],'rows');
MP=LSL(idxOrigin,:);
end

