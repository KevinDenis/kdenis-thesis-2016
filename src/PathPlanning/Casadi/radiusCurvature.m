function [curv,r,cx,cy] = radiusCurvature(xx, yy)
curv=zeros(size(xx));
r=zeros(size(xx));
cx=zeros(size(xx));
cy=zeros(size(xx));
for ii=1:length(xx)
    if ii==1
        x=xx(ii:ii+2);
        y=yy(ii:ii+2);
    elseif ii>length(xx)-1
        x=xx(ii-2:end);
        y=yy(ii-2:end);
    else
        x=xx(ii-1:ii+1);
        y=yy(ii-1:ii+1);
    end
    mx = mean(x); my = mean(y);
    X = x - mx; Y = y - my; % Get differences from means
    dx2 = mean(X.^2); dy2 = mean(Y.^2); % Get variances
    t = [X,Y]\(X.^2-dx2+Y.^2-dy2)/2; % Solve least mean squares problem
    a0 = t(1); b0 = t(2); % t is the 2 x 1 solution array [a0;b0]
    r(ii) = sqrt(dx2+dy2+a0^2+b0^2); % Calculate the radius
    cx(ii) = a0 + mx; 
    cy(ii) = b0 + my; % Locate the circle's center
    curv(ii) = 1/r(ii); % Get the curvature

end

end % function radiusCurvature
