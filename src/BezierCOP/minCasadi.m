function [minVal,idx] = minCasadi(x)
%MINCASADI Summary of this function goes here
%   Detailed explanation goes here
minVal = x(1);
for i=2:length(x)
    minVal = min(minVal, x(i));
end
idx = find(x == minVal);
idx=idx(1);
end

