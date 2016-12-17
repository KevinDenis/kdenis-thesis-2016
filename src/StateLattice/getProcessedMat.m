function [A_norm_unique] = getProcessedMat(A)
%GETPROCESSEDMAT Summary of this function goes here
%   Detailed explanation goes here
A_norm=normr(A);
A_norm_round=round(A_norm,1);
A_norm_unique = unique(A_norm_round,'rows');
end
