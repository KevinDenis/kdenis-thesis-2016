function [LSL_Bcw] = getBackwardMotionFromStateLattice(LSL)
%[LSL_Fwd] = getForwardMotionFromStateLattice(LSL)
LSL_Bcw=LSL(all([[LSL.x1]<=0;[LSL.x0]<=0],1));
end

