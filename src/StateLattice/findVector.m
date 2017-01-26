function [idx] = findVector(A,b)
%FINDVECTOR Summary of this function goes here
%   Detailed explanation goes here

[err,idx]=min(sum((A-kron(ones(size(A,1),1),b)).^2,2));
if err ~= 0
    disp(['WARNING : ERROR OF ',num2str(err)])
end
end

