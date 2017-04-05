function [idx] = findVector(A,b)
%FINDVECTOR Summary of this function goes here
%   Detailed explanation goes here
[~,n_A]=size(A);
[m,n_b]=size(b);

if n_A == n_b
    idx=zeros(m,1);
    for ii=1:m
        b_ii=b(ii,:);
        idx_ii=findrow_mex(A,b_ii);
        if isnan(idx_ii)
            b_ii_rev = [b_ii(4:end) b_ii(1:3)];
            idx_ii=findrow_mex(A,b_ii_rev);
        end
        idx(ii)=idx_ii;
    end
end
