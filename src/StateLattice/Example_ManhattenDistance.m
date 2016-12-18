clear 
close all
clc

X_max=2;
dx=0.1;
X_vec=-X_max:dx:X_max;
Y_vec=X_vec;

TH_vec=(0:360/40:360-1);

origin_XkYk=[find(X_vec==0),find(Y_vec==0)];

grid_XY=getAllComb(X_vec,Y_vec);

grid_XY_L1 = abs(grid_XY(:,1))+abs(abs(grid_XY(:,2)));

figure()
hold on
scatter(grid_XY(:,1),grid_XY(:,2))

for i=1:50
scatter(grid_XY(IsNear(grid_XY_L1,i*dx,1e-3),1),grid_XY(IsNear(grid_XY_L1,i*dx,1e-3),2),'filled')
drawnow
end