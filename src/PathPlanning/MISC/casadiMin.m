function [m] = casadiMin(A)
%CASADIMIN Summary of this function goes here
%   Detailed explanation goes here
m=A(1);
for ii=1:numel(A)
    if m > A(ii)
        m=A(ii);
    end
end
end

