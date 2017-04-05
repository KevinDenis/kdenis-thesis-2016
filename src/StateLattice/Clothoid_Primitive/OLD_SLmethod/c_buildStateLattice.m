% Notes :
% always keep matrix and structure, because it is much faster to search
% through a matrix

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                      %
%               Motion Primitive Matrix                %
%  [ x0 y0 th0 x1 y1 th1 kappa dkappa Ltot intKappa ]  %
%    1  2   3  4  5   6    7     8    9    10          %
%                                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                               %
%                    State Latice Structure                     %
%   [ x0 y0 th0 x1 y1 th1 X Y TH kappa dkappa Ltot intKappa ]   %
%                                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


clearvars -except MotionPrem
if ~exist('MotionPrem', 'var')
    load('MotionPrem.mat');
end
co=get(gca,'ColorOrder'); % get default color for plot
close all
clc
addpath('ClothoidG1fitting');

dx_shift=1;
x_shift_max=4;

x_sift_vec=-x_shift_max:dx_shift:x_shift_max;
y_sift_vec=x_sift_vec;

growthFactor=length(x_sift_vec)*length(y_sift_vec);
lengthMP=length(MotionPrem);
lengthSL=growthFactor*lengthMP;
StateLattice=MotionPrem(1);
StateLattice(lengthSL)=MotionPrem(end);
StateLattice=StateLattice.';

kk=1:length(MotionPrem);
for ii=1:growthFactor
    StateLattice(kk)=MotionPrem;
   kk=kk+lengthMP;
end
counter=0;
progressbar('Building State Lattice')
kk=1:length(MotionPrem);
for x_shift=x_sift_vec
    for y_shift=y_sift_vec
        for ii=kk
            StateLattice(ii).x0=StateLattice(ii).x0+x_shift;
            StateLattice(ii).y0=StateLattice(ii).y0+y_shift;
            StateLattice(ii).x1=StateLattice(ii).x1+x_shift;
            StateLattice(ii).y1=StateLattice(ii).y1+y_shift;
            StateLattice(ii).X=StateLattice(ii).X+x_shift;
            StateLattice(ii).Y=StateLattice(ii).Y+y_shift;
            StateLattice(ii).PathOccXY(:,1)=StateLattice(ii).PathOccXY(:,1)+x_shift;
            StateLattice(ii).PathOccXY(:,2)=StateLattice(ii).PathOccXY(:,2)+y_shift;
            counter=counter+1;
            StateLattice(ii).ID=counter;
            progressbar(counter/lengthSL)
        end
        kk=kk+length(MotionPrem);
    end
end


% save('StateLattice.mat','StateLattice')
%}