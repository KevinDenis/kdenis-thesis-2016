function [ dydx ] = dydx(P)
%DYDX Summary of this function goes here
%   Detailed explanation goes here
[m,n]=size(P);

if n~=2
   disp('wrong input')
   return
end

dydx=P(:,2)./P(:,1);

end

