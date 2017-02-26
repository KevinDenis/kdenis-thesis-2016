clear
close all
clc

X=[0 1 2]';
Y=[0 0 1]';
PNTS=[X Y];

[TH,k,dk,L,nevalG1,~,~,~,~] = G1spline(PNTS);

figure(1)
scatter(X,Y)
hold on
for ii = 1:length(k)
    [Xsol,Ysol] = pointsOnClothoid(X(ii),Y(ii),TH(ii),k(ii),dk(ii),L(ii),1000);
    plot(Xsol,Ysol,'b')
end


