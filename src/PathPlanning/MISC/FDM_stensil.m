clear
close all
clc

[x,y]=ginput(4)
    

P=BezierCurve(x,y);
xx=P(:,1);
yy=P(:,2);

for i=1:length(xx)-1
    h=abs(xx(i)-xx(i+1));
end


figure(1)
% plot(xx,fdm(xx,yy,1,2)); 
plot(xx,fdm(xx,yy,1,5)); hold on
% plot(xx,fdm(xx,yy,1,10)); hold on
plot(xx,gradient(yy,xx))
plot(xx(2:end),diff(yy)./h)
% plot(xx,fdm(xx,yy,1,10)); hold on

