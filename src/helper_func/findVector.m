function [idx] = findVector(A,b)
%[idx] = findVector(A,b)
[~,n_A]=size(A);
[m,n_b]=size(b);

if n_A == n_b
    idx=zeros(1,1);
    nn=1;
    for ii=1:m
        b_ii=b(ii,:);
        idx_ii=findrow_mex(A,b_ii);
        if ~isnan(idx_ii)
            idx(nn)=idx_ii;
            nn=nn+1;
        end
    end
end
