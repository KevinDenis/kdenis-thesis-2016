clear
close all
clc


%% Init
% General parameters
x1=0; y1=0; th1=pi/2; Pstart=[x1,y1,th1]; % start position and orientation
xend=2; yend=2; thend=pi/4; Pend=[xend,yend,thend]; % end position and orientation
n=3; % order of the bezier curve
t=linspace(0,1,101)'; % overall resolution of the solver
kappa_max=1;
% Weigting factor for objective function
% these will be normalized (for conviniance) and rounded off and then
% checked for uniqueness
a_kappa_mat=1;
a_length_mat=0;

weights_mat=[a_kappa_mat' a_length_mat'];
weights_mat=getProcessedMat(weights_mat);
weights=weights_mat(1,:);

initCOP=struct('Pstart',Pstart,'Pend',Pend,'n',n,'t',t,'weights',weights,'kappa_max',kappa_max);

x1=0;
xend=1;
y1=0;
yend=1;
disc=0.02;

X_vec=x1+0.1:disc:xend-0.1;
Y_vec=y1+0.1:disc:yend-0.1;
[X,Y] = ndgrid(X_vec,Y_vec);
kappa_res=zeros(size(X));
for ii=1:length(X_vec)
    for jj=1:length(Y_vec)
        x_ii=[x1;X_vec(ii);xend];
        y_jj=[y1;Y_vec(jj);yend];
        kappa_res(ii,jj)=objectiveFunction(x_ii,y_jj,initCOP);
        plot(X(:,jj),kappa_res(:,jj))
        drawnow
    end
end
% plot(X_vec,kappa_res)
mesh(X,Y,kappa_res)
% jj=23
slice
% plot(X(:,jj),kappa_res(:,jj))