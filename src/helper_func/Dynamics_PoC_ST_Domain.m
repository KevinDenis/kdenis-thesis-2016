%Ex3
clear; close all; clc;

tic
N = 100;      %discretization points
c = 2;        %drag coefficient[Ns^2/m^2]
m = 500;      %car mass [kg]
d = 5;     %distance to travel[m]
Fmin = -500; %minimum force[N]
Fmax =  500; %maximum force [N]

obs = [ 2.75 1.75;
     	3.25 1.75;
        3.75 2;
        3.25 2.25;
        2.75 2.25;
        2.75 1.75];

% Ode description of car model
ode = @(x,u) [x(2);(u-c*x(2).*x(2))/m];

% Decision variables
s = optivar(2,N+1); %states [x, x_dot]
a = optivar(N,2);
b = optivar(N,1);
F = optivar(N,1);   %input
T = optivar();      %motion time

h = T/N; %discretization step

%build constraints
con = {}; 
for k=1:N
%     xp = s(:,k) + h*ode(s(:,k),F(k)); %Euler integration
    xp = rk_step(ode,s(:,k),F(k),h); %Runge-Kutta integration
    con = {con{:}, xp == s(:,k+1)};   %impose system dynamics
    con = {con{:}, a(k,:)*[h*k s(1,k)].' - b(k,:) <= -0.1};
    for ii=1:(length(obs)-1)
        con = {con{:}, a(k,:)*obs(ii,:).' - b(k,:) >= 0};
    end
    con = {con{:}, a(k,:)*a(k,:).' <= 1};
end

con = {con{:}, F<= Fmax, F>=Fmin};%force constraints
con = {con{:}, s(:,1)==[0;1] , s(1,end)==[d]}; %initial and final conditions
con = {con{:}, T>=0}; %positive motion time
con = {con{:}, s(2,:)>=0}; %positive velocity = forwards driving
con = {con{:}, s(2,:)<=1}; %positive velocity = forwards driving


%necessary?
T.setInit(50); %assign initial guess to T
s.setInit([linspace(0,d,N+1);zeros(1,N+1)]); %assign initial guess to states
F.setInit(ones(N,1)); %initial guess for force
 
optisolve(T, con); %solve optimization problem
toc
ts = linspace(0,optival(T),N+1);

xsol = optival(s); %get value of casADi variable

%plot results
figure(1)
hold on
plot (ts, xsol(1,:))
plot(obs(:,1),obs(:,2))
xlabel('Time [s]')
ylabel('Distance [m]')

% figure(2)
% plot (ts, xsol(2,:))
% xlabel('Time [s]')
% ylabel('Velocity [m/s]')
% 
% figure(3)
% hold on
% plot(ts(1:end-1) , optival(F),'b') 
% plot (ts, Fmax*ones(size(ts,2)),'r-')
% plot (ts, Fmin*ones(size(ts,2)),'r-')
% xlabel('Time [s]')
% ylabel('Force [N]')

disp(strcat('Optimal motion time: ' , num2str(optival(T)), ' s'));