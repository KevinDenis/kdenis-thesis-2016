function [C1,C2] = casteljau(Z,s)
%CASTELJAU(Z,s) De Casteljau method.
% CASTELJAU(Z,s) is the Bezier curve defined by Z at t=s
n = size(Z,1);
b1 = x;
b2 = y;
for k = 2:n
    for r = n:-1:k
        b1(r) = b1(r-1) + s * ( b1(r) - b1(r-1) );
        b2(r) = b2(r-1) + s * ( b2(r) - b2(r-1) );
    end
end
C1 = b1(n);
C2 = b2(n);
end