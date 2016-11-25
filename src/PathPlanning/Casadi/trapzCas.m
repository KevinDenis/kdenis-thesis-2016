function z = trapz(x,y)
%TRAPZ  Trapezoidal numerical integration.
%   Z = TRAPZ(Y) computes an approximation of the integral of Y via
%   the trapezoidal method (with unit spacing).  To compute the integral
%   for spacing different from one, multiply Z by the spacing increment.
%
%   For vectors, TRAPZ(Y) is the integral of Y. For matrices, TRAPZ(Y)
%   is a row vector with the integral over each column. For N-D
%   arrays, TRAPZ(Y) works across the first non-singleton dimension.
%
%   Z = TRAPZ(X,Y) computes the integral of Y with respect to X using
%   the trapezoidal method.  X and Y must be vectors of the same
%   length, or X must be a column vector and Y an array whose first
%   non-singleton dimension is length(X).  TRAPZ operates along this
%   dimension.
%
%   Z = TRAPZ(X,Y,DIM) or TRAPZ(Y,DIM) integrates across dimension DIM
%   of Y. The length of X must be the same as size(Y,DIM)).
%
%   Example:
%       Y = [0 1 2; 3 4 5]
%       trapz(Y,1)
%       trapz(Y,2)
%
%   Class support for inputs X, Y:
%      float: double, single
%
%   See also SUM, CUMSUM, CUMTRAPZ, INTEGRAL.

%   Copyright 1984-2015 The MathWorks, Inc.

%   Make sure x and y are column vectors, or y is a matrix.

perm = []; nshifts = 0;
if nargin == 3 % trapz(x,y,dim)
  perm = [dim:max(ndims(y),dim) 1:dim-1];
  y = permute(y,perm);
  m = size(y,1);
elseif nargin==2 && isscalar(y) % trapz(y,dim)
  dim = y; y = x;
  perm = [dim:max(ndims(y),dim) 1:dim-1];
  y = permute(y,perm);
  m = size(y,1);
  x = 1:m;
else % trapz(y) or trapz(x,y)
  if nargin < 2, y = x; end
  [y,nshifts] = shiftdim(y);
  dim = nshifts + 1;
  m = size(y,1);
  if nargin < 2, x = 1:m; end
end
if ~isvector(x)
  error(message('MATLAB:trapz:xNotVector'));
end
x = x(:);
if length(x) ~= m
    error(message('MATLAB:trapz:LengthXmismatchY',dim));
end

% The output size for [] is a special case when DIM is not given.
if isempty(perm) && isequal(y,[])
  z = zeros(1,class(y));
  return;
end

%   Trapezoid sum computed with vector-matrix multiply.
z = diffMatrix(x).' * (y(1:m-1,:) + y(2:m,:))/2;

siz = size(y); siz(1) = 1;
z = reshape(z,[ones(1,nshifts),siz]);
if ~isempty(perm), z = ipermute(z,perm); end
