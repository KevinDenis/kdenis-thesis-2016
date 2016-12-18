function d = Norm2(A)
%NORM2 Summary of this function goes here
%   Detailed explanation goes here
d=A(:,1);
for ii=1:length(A(:,1))
    d(ii)=sqrt(A(ii,:)*A(ii,:)');
end
end

