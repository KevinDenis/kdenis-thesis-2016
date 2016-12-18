clear
close all
clc

stateLatice=[];
x_max=2;
dx=0.5;
X_vec=0:dx:x_max;
Y_vec=X_vec;
dth=pi/2;
TH1_vec=-pi/2:dth:pi/2;
THend_vec=TH1_vec;
grid_TH1_X_Y_THend=getAllComb(TH1_vec,X_vec,Y_vec,THend_vec);
grid_X_Y_L1 = abs(grid_TH1_X_Y_THend(:,2))+abs(abs(grid_TH1_X_Y_THend(:,3)));

progressbar('Overall State Lattice Progress','Current Manhatten Distance Progress')

progressbar('Overall State Lattice Progress','Current Manhatten Distance Progress')
for ii=1/dx:2*x_max/dx

    kk=IsNear(grid_X_Y_L1,ii*dx,1e-3);
    grid_kk=grid_TH1_X_Y_THend(kk,:);
    for jj=1:size(grid_kk,1)
        pause(0.01) % Do something important
        % Update all bars
        frac2 = jj/size(grid_kk,1);
        frac1 = (ii-1/dx+frac2)/(2*x_max/dx-1/dx+1);
        progressbar(frac1, frac2)
    end
end
