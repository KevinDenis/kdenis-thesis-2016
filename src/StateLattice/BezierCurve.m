function [B,dB,ddB,kappa]=BezierCurve(x,y,varargin)

if nargin == 2
    t = linspace(0,1,101)'; 
elseif isscalar(varargin{1})
    t=linspace(0,1,varargin{1})';
elseif isvector(varargin{1})
    t=varargin{1};
end

[nx,mx]=size(x);
[ny,my]=size(y);
if mx ~=1 && nx == 1
    x=x';
    nx=mx;
end
if my ~=1 && ny == 1
    y=y';
end

n=nx-1; % n = order of the bézier curve = # controlpoints - 1

if length(x) ~= length(y)
    disp('x and y dont have the same length.')
    return
end

dx=diffMatrix(x);
dy=diffMatrix(y);
ddx=diffMatrix(dx);
ddy=diffMatrix(dy);

B=bernsteinMatrix(n, t)*[x y];
dB=n*bernsteinMatrix(n-1,t)*[dx dy];
ddB=(n-1)*(n)*bernsteinMatrix(n-2,t)*[ddx ddy];

dxx=dB(:,1);
dyy=dB(:,2);
ddxx=ddB(:,1);
ddyy=ddB(:,2);

kappa=((dxx.*ddyy-dyy.*ddxx))./((dxx.^2+dyy.^2).^(3/2));
end
