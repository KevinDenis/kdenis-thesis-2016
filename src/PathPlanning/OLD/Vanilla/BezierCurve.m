function [B,dB,ddB,kappa]=BezierCurve(P,varargin)

if nargin == 1
    t = linspace(0,1,101)'; 
elseif isscalar(varargin{1})
    t=linspace(0,1,varargin{1})';
elseif isvector(varargin{1})
    t=varargin{1};
end

[nn,mm]=size(P);
n=nn-1; % n = order of the b�zier curve = # controlpoints - 1

if mm ~= 2
    disp('please write your point this way [x_i y_i]')
    return
end

x=P(:,1);
y=P(:,2);
dP=[diffMatrix(x) diffMatrix(y)];
ddP=[diffMatrix(diffMatrix(x)) diffMatrix(diffMatrix(y))];



B=bernsteinMatrix(n, t)*P;
dB=n*bernsteinMatrix(n-1,t)*dP;
ddB=(n-1)*(n)*bernsteinMatrix(n-2,t)*ddP;

dx=dB(:,1);
dy=dB(:,2);
ddx=ddB(:,1);
ddy=ddB(:,2);

kappa=((dx.*ddy-dy.*ddx))./((dx.^2+dy.^2).^(3/2));
end
