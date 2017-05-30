function [ ReducedStateLattice ] = ReducePathDensity(StateLattice,LSLset)
% [ ReducedStateLattice ] = ReducePathDensity(StateLattice,LSLset)
dth=2*LSLset.dth;

n=length(StateLattice);
toKeep=zeros(1);
jj=1;
for ii=1:n
    th0_ii=StateLattice(ii).th0;
    th1_ii=StateLattice(ii).th1;
    if IsNear(rem(th0_ii,dth),0,1e-2) && IsNear(rem(th1_ii,dth),0,dth/4)
        toKeep(jj) = ii;
        jj=jj+1;
    end
end

ReducedStateLattice=StateLattice(toKeep);
end