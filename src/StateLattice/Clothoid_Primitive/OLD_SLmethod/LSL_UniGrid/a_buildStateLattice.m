% Notes :
%   *
%   *

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%                        State Latice Structure                           %
%[ x0 y0 th0 x1 y1 th1 X Y TH k dk Ltot intK PathOccXY pathCost free ID ] %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%#ok<*NBRAK>

clearvars
close all
clc
addpath('ClothoidG1fitting');

%% Init
% General parameters
showPlot=false;

MP_par.x0=0; 
MP_par.y0=0;
MP_par.th0=0;
MP_par.x_max=2;
MP_par.y_max=1.5;
MP_par.k_max=1;
MP_par.res = 5e-2;
MP_par.dx=0.25;
MP_par.dth=pi/8;
dx_SL=2*MP_par.dx;

[MotionPrem] = getMotionPrem(MP_par);
MP_par_mod=MP_par;
StateLattice=MotionPrem;
n=length(MotionPrem);
kk=n;
for ii=1:n
    x1_ii=MotionPrem(ii).x1;
    y1_ii=MotionPrem(ii).y1;
    th1_ii=MotionPrem(ii).th1;
    if rem(x1_ii,dx_SL) == 0 && rem(y1_ii,dx_SL) == 0
        MP_par_mod.x0=x1_ii;
        MP_par_mod.y0=y1_ii;
        MP_par_mod.th0=th1_ii;
        [MotionPremRotTrans] = getMotionPrem(MP_par_mod);
        kk=kk(end)+[1:length(MotionPremRotTrans)];
        StateLattice(kk)=MotionPremRotTrans;
    end
end

StateLattice=CleanupStateLattice(StateLattice);

%% Plot Motion Primitive
if showPlot
    figure()
    hold on
    grid on
    xlabel('x [m]')
    ylabel('y [m]')
    axis equal
    plotPath(StateLattice)
end