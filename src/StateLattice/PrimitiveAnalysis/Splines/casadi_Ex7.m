%Ex7

clear; close all; clc;

tic
N = 200;      %discretization points
c = 2;        %drag coefficient[Ns^2/m^2]
m = 500;      %car mass [kg]
d = 1000;     %distance to travel[m]
Fmin = -2500; %minimum force[N]
Fmax =  2500; %maximum force [N]

%Initial conditions
x0 = 0;
dx0 = 0;

%Set up splines
deg = 3;      % spline degree
L   = d;      % range [0 , L]
n   = 20;     % number of knots
Bl  = BSplineBasis([0 , L], deg, n); %make B-spline basis
b   = BSpline.optivar(Bl, [1,1]); 
db  = b.derivative(1);

%Initial guess
Tvar = 50;
b_avg = ((d/N)/Tvar).^2;
bvar = [0, b_avg*ones(1, Bl.length-2), 0];

%Set up objective
T = 0;
for k = 1:N
    T = T + (2*d/N) / (sqrt(2*eps+b.f(d*(k+1)/(N+1))) + sqrt(2*eps+b.f(d*k/(N+1)))); %add 2*eps to avoid numerical problems due to b(0) and b(d) = 0
end %note that in b.f(d*(k+1)/(N+1)), d is required to evaluate the full spline, dividing by N+1 is necessary to get full evaluation of the spline until d

%Set up constraints
con = {};
con = {con{:}, b.f(0) == 0, b.f(d) == 0};
con = {con{:}, b >= 0};
con = {con{:}, m * 1/2 * db + c * b <= Fmax};
con = {con{:}, m * 1/2 * db + c * b >= Fmin};

b.coeffs.data.setInit(bvar);

optisolve(T,con);
toc

b = BSpline(Bl, b.coeffs);%use spline coefficients to build the motion trajectory
b = optival(b);           %convert casADi variable to access its numerical value

%Remove negative values of b
b.coeffs(:) = max(b.coeffs.data,0); %first and last coefficient are negative numbers very close to 0, make them exactly 0 to avoid problems when taking sqrt
db = b.derivative(1);

%Build up time vector, according to the proposed discretization
T = 0;
tvec = [0];
for k = 1:N
    T = T + (2*d/N) / (sqrt(b.f(d*(k+1)/(N+1))) + sqrt(b.f(d*k/(N+1))));
    tvec = [tvec , T];
end

solT = optival(T);

%Integrate b to get position
pos = 0; %start position/integration constant
for i = 1:N %start at 1 since pos at time 0 is included via start position
%     pos = [pos, pos(end) + (tvec(i+1)-tvec(i)) * sqrt(b.f(d*(i)/(N)))]%Euler integration,
%     gives bad result because in the last iteration you multiply by 0,
%     since b.f(d)=0 --> use better integration rule
    pos = [pos, pos(end) + (tvec(i+1)-tvec(i)) * (sqrt(b.f(d*(i+1)/(N+1)))+sqrt(b.f(d*(i)/(N+1))))/2]; %trapezium rule
end

s = [0:0.01:1]; 
figure(1)
plot(tvec, pos) %plot position w.r.t. time
xlabel('Time [s]')
ylabel('Distance [m]')
figure(2)
plot(s*d, sqrt(b.f(d*s))) %remark: we now plot w.r.t. distance, since b is function of x
xlabel('Distance [m]')
ylabel('Velocity [m/s]')
figure(3)
plot(s*d, 1/2*db.f(d*s))
xlabel('Distance [m]')
ylabel('Acceleration [m/s^2]')
figure(4)
hold on
plot(s*d, m * 1/2 * db.f(d*s) + c * b.f(d*s))
plot (s*d, Fmax*ones(size(s,2)),'r-')
plot (s*d, Fmin*ones(size(s,2)),'r-')
xlabel('Distance [m]')
ylabel('Force [N]')

disp(strcat('Optimal motion time: ' , num2str(optival(T)), ' s'));