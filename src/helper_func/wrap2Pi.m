function [ lambda ] = wrap2Pi( lambda )
%wrap2Pi Wrap angle in radians to [-pi+eps pi]
%
%   lambdaWrapped = wrap2Pi(LAMBDA) wraps angles in LAMBDA, in radians,
%   to the interval [-pi+eps pi] such that pi maps to pi and -pi maps to
%   -pi.  (In general, odd, positive multiples of pi map to pi and odd,
%   negative multiples of pi map to -pi.)
%
%   See also wrapTo2Pi, wrapTo180, wrapTo360.

% Copyright 2007-2008 The MathWorks, Inc.
% Small modification by K.Denis : -pi becomes pi

q = (lambda < -pi) | (pi < lambda);
lambda(q) = wrapTo2Pi(lambda(q) + pi) - pi;
p = lambda == -pi;
lambda(p)=pi;
end

