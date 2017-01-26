%Ex7

% clear; close all; clc;

tic


%Initial conditions
s=0:0.01:1;

%Set up splines
deg = 4;      % spline degree
L   = 1;      % range [0 , L]
n   = 5;     % number of knots
Bl  = BSplineBasis([0 , L], deg, n); %make B-spline basis
S   = BSpline.optivar(Bl, [1,1]); 
dS  = S.derivative(1);
ddS  = S.derivative(2);

%set up objective
obj = ddS.integral;


%Set up constraints
con = {};
con = {con{:}, S.f(0) == 0, S.f(1) == 0, S.f(0.5)==0.5};
con = {con{:}, dS.f(0) == 0, dS.f(1) == 0};
con = {con{:}, S >= 0};
con = {con{:}, ddS <= 10};
con = {con{:}, ddS >= -10};
optisolve(obj,con);
toc

% S = BSpline(Bl, S.coeffs);%use spline coefficients to build the motion trajectory
S = optival(S);           %convert casADi variable to access its numerical value

hold on
plot(s,S.f(s))
% plot(s,S.derivative(2).f(s))


%{
s = [0:0.01:1]; 
figure(1)
plot(tvec, pos) %plot position w.r.t. time
xlabel('Time [s]')
ylabel('Distance [m]')
figure(2)
plot(s*d, sqrt(S.f(d*s))) %remark: we now plot w.r.t. distance, since b is function of x
xlabel('Distance [m]')
ylabel('Velocity [m/s]')
figure(3)
plot(s*d, 1/2*dS.f(d*s))
xlabel('Distance [m]')
ylabel('Acceleration [m/s^2]')
figure(4)
hold on
plot(s*d, m * 1/2 * dS.f(d*s) + c * S.f(d*s))
plot (s*d, Fmax*ones(size(s,2)),'r-')
plot (s*d, Fmin*ones(size(s,2)),'r-')
xlabel('Distance [m]')
ylabel('Force [N]')

disp(strcat('Optimal motion time: ' , num2str(optival(T)), ' s'));
%}