function [LSL_Fwd] = getForwardMotionFromStateLattice(LSL)
%[LSL_Fwd] = getForwardMotionFromStateLattice(LSL)
LSL_Fwd=LSL(all([[LSL.x1]>=0;[LSL.x0]>=0],1));
LSL_Fwd=removeToSFromLSL(LSL_Fwd);
end

