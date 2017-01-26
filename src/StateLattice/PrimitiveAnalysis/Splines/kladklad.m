%Ex6

clear all; close all; clc;

tic
d = 1;     %distance to travel[m]

%Initial conditions
x0 = 0;
dx0 = 0;
theta0=0;
theta1=0;
s = 0:0.01:1; %create time vector: go from 0 to 1 in steps of 0.01s

%Set up splines
deg = 3;      % spline degree
L   = 1;      % range [0 , L]
n   = 30;     % number of knots
Bl  = BSplineBasis([0 , L], deg, n);  %make B-spline basis
xy   = BSpline.optivar(Bl, [1, 2]);    %variable: movement spline. Shape/Size: 1 by 2 (= [x , y])
dxy  = xy.derivative(1);                %velocity spline
ddxy = xy.derivative(2);
dx  = dxy(1);       %velocity spline
ddx = ddxy(1);
dy  = dxy(2);           %velocity spline
ddy = ddxy(2);

%Initial guess
xvar = [linspace(x0,d,Bl.length)'];
yvar = [linspace(x0,d,Bl.length)'];

%Set up optimization problem
ds=dy*dy+dx*dx;
obj = ds.integral; %objective function: motion time
    
%Define constraints: initial and final conditions, kinematic constraints motion time constraint
con = {};
con = {con{:}, xy(1).f(0) == x0, xy(1).f(1) == d}; %initial position and goal position
con = {con{:}, xy(2).f(0) == x0, xy(2).f(1) == d};
con = {con{:}, dxy(2).f(0)/dxy(1).f(0) == theta0, dxy(2).f(1)/dxy(1).f(1) == theta1};

xy.coeffs.data.setInit([xvar yvar]);         %assign initial guess to coeffs of x

%Solve optimization problem
optisolve(obj,con);          %solve optimization problem

%Process solution
xy = BSpline(Bl, xy.coeffs);%use spline coefficients to build the motion trajectory
xy = optival(xy);           %convert casADi variable to access its numerical value
dxy = xy.derivative(1);     %calculate velocity
ddxy = xy.derivative(2);    %calculate acceleration

%Plot solutions
figure(1)     
vehicle_path = plot(xy(1).f(s),xy(2).f(s), 'b');%plot motion trajectory
xlabel('Time [s]')
ylabel('Position [m]')