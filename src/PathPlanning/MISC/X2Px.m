function Px = X2Px(X)
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

selectY=diag(flip(OneZero));

selectY( ~any(selectY,2), : ) = []; % remove 0 rows

% P=zeros(4,2);
Px=selectY*X;

end

