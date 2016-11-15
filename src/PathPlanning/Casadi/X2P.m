function [P] = X2P(X)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

[m,n]=size(X);

if n~=1 && rem(m,2)~=0
   disp('wrong input')
   return
end

OneZero=zeros(m/2,1);
for ii=1:2:m
    OneZero(ii:ii+1)=[1 0];
end

selectX=diag(OneZero);
selectY=diag(flip(OneZero));

selectX( ~any(selectX,2), : ) = []; % remove 0 rows
selectY( ~any(selectY,2), : ) = []; % remove 0 rows

% P=zeros(4,2);
P(:,1)=selectX*X;
P(:,2)=selectY*X;


end

