function [y_init] = getInitValY(y1,yg,n)
%GETINITVAL Summary of this function goes here
%   Detailed explanation goes here
% y_init=[linspace(y1,yg,n)]';
y_init=zeros(n,1);
d=yg-y1;
y_init(1)=y1;
y_init(end)=yg;
if rem(n,2) == 0
    y_init(2:n/2)=y1;
    y_init(n/2+1:end)=yg;
else
    y_init(2:(n-1)/2)=y1;
    y_init((n+1)/2)=d/2;
    y_init((n+3)/2:end)=yg;
end
y_init=zeros(n,1);
end

