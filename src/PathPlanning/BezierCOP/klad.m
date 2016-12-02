clear
close all
clc
% import casadi.*


% x = MX.sym('x',4,1);


% size(x)
x2=1;
x3=-1:0.01:3;
x1=0;
x4=2;
x=[x1;x2;x3;x4];

y2=-1:0.2:3;
y3=-1:0.2:3;
y1=zeros(size(x2));
y4=2*ones(size(x2));
y=[y1;y2;y3;y4];

% optMat=getOptMat(x1(1),x2,x3,x4(1),y1(1),y2,y3,y4(1));

Pstart=[x1(1),y1(1),0]; % start position and orientation
Pgoal=[x4(1),y4(1),0]; % end position and orientation
n=4; % order of the bézier curve
t=linspace(0,1,101)'; % overall resolution of the solver
kappa_max=5;
% Weigting factor for objective function
% these will be normalized (for conviniance) and rounded off and then
% checked for uniqueness
a_kappa_mat=    [ 2.0  1.0];
a_length_mat=   [ 1.0  1.0];
a_dist_mat=     [ 5.0  5.0];
a_angle_mat=    [ 0.0  0.5];
weights_mat=[a_kappa_mat' a_length_mat' a_dist_mat' a_angle_mat'];
weights_mat=getProcessedMat(weights_mat);
% weights=optipar(1,4);
% weights.setValue(weights_mat(1,:))
weights=weights_mat(1,:);

[bezierMatrix,zeroToZMatrix,zToTMatrix]=getBezierMatrix(n-1,0.5);

obs=[1.0 0.5;1.5 0.5;1.5 0.0;1.0 0.0];

initCOP=struct('Pstart',Pstart,'Pgoal',Pgoal,'n',n,'t',t,'weights',weights,'kappa_max',kappa_max,'obs',obs,'zeroToZMatrix',zeroToZMatrix,'zToTMatrix',zToTMatrix);
f=zeros(length(optMat),1);
parfor ii=1:length(optMat)
    x_array=optMat(ii,1:4);
    y_array=optMat(ii,5:8);
    f(ii)=objectiveFunction(x_array',y_array',initCOP);
end

