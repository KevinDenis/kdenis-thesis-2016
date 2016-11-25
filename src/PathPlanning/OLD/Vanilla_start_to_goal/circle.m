function [xp,yp]=circle(x,y,r,co,plot)
%circle(x,y,r,co)
%   x and y are the coordinates of the center of the circle
%   r is the radius of the circle
%   0.01 is the angle step, bigger values will draw the circle faster but
%   you might notice imperfections (not very smooth)
ang=0:0.1:2*pi; 
xp=r*cos(ang);
yp=r*sin(ang);
if plot
    plot(x+xp,y+yp,'Color',co);
end
end