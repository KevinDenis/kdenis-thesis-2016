initWorkspace
co=get(groot,'DefaultAxesColorOrder');
load('matlab.mat')

Path_S = Path.S;
Path_X = Path.X;
Path_Y = Path.Y;
Path_TH = Path.TH;

t_sim = (0:0.01:t_sol(end)).';

% V_c_obs_circ=1.5*V_c_obs_circ;

s_sol(2,end)=s_sol(2,end-1);
s_sim = interp1(t_sol,s_sol(1,:),t_sim);
v_sim = interp1(t_sol,s_sol(2,:),t_sim);

X_sim = interp1(Path_S,Path_X,s_sim);
Y_sim = interp1(Path_S,Path_Y,s_sim);
TH_sim = interp1(Path_S,Path_TH,s_sim);

C_xy_sim_rec = repmat(V_c_obs_rec,length(t_sim),1).*repmat(t_sim,1,2)+repmat(C_xy_start_obs_rec,length(t_sim),1);
C_xy_sim_circ = repmat(V_c_obs_circ,length(t_sim),1).*repmat(t_sim,1,2)+repmat(C_xy_start_obs_circ,length(t_sim),1);

% X_sim = interp1(
for ii = 1:length(t_sim)
if ii == 1
    fig=figureFullScreen(1);
% else
%     figure(1)
end
clf
hold on
plotPath(Path,co(1,:),2)
plotRoboticWheelchair([X_sim(ii),Y_sim(ii),TH_sim(ii)],'k',3)
plotRectangle([C_xy_sim_rec(ii,:),atan2(V_c_obs_rec(2),V_c_obs_rec(1))],co(2,:),3)
plotCircle([C_xy_sim_circ(ii,:),atan2(V_c_obs_circ(2),V_c_obs_circ(1))],co(3,:),3)
text(2, .0,['Wheelchair velocity: ',num2str(v_sim(ii),'%1.2f'), ' m/s'],'FontSize',16)
text(2,-.1,['Distance along path: ',num2str(s_sim(ii),'%1.1f'), ' m'],'FontSize',16)
text(2,-.2,['          Time elapsed: ',num2str(t_sim(ii),'%1.1f'), ' s'],'FontSize',16)
axis([-1.25 3 -0.5 2.2])
axis normal
hold off
xlabel('x [m]')
ylabel('y [m]')
% drawnow
DVP_Animation(ii) = getframe;  % save frame to a variable Film
end

save('DVP_Animation.mat','DVP_Animation')

figureFullScreen(2);
movie(DVP_Animation,1,36)
myVideo = VideoWriter('DVP_Animation.mp4','MPEG-4');
open(myVideo);
writeVideo(myVideo, DVP_Animation);
close(myVideo)

function plotSpeedometer(v)
zero_speed_angle = 225*pi/180;
one_speed_angle = 135*pi/180;
half_speed_angle = (zero_speed_angle+one_speed_angle)/2;

% speed indicator
x_ind_s = -0.5;
y_ind_s = 2;
r_ind = 0.35;
th_ind = interp1([0,1],[zero_speed_angle one_speed_angle],v);
x_ind_e = x_ind_s + r_ind*cos(th_ind);
y_ind_e = y_ind_s + r_ind*sin(th_ind);
x_ind = [x_ind_s x_ind_e];
y_ind = [y_ind_s y_ind_e];

% circular arc
th_gauge = zero_speed_angle:-0.01:one_speed_angle-0.1;
r_gauge = 0.85*r_ind;
r_gauge_ind = 1.1*r_ind;
x_gauge = x_ind_s + r_gauge*cos(th_gauge);
y_gauge = y_ind_s + r_gauge*sin(th_gauge);
x_gauge_0 = [x_ind_s, x_ind_s + r_gauge_ind*cos(zero_speed_angle)];
y_gauge_0 = [y_ind_s, y_ind_s + r_gauge_ind*sin(zero_speed_angle)];
x_gauge_1 = [x_ind_s, x_ind_s + r_gauge_ind*cos(one_speed_angle)];
y_gauge_1 = [y_ind_s, y_ind_s + r_gauge_ind*sin(one_speed_angle)];
% text indicator

plot(x_ind,y_ind,'Color','k','LineWidth',3)
plot(x_gauge_0,y_gauge_0,'--','Color','k','LineWidth',1)
plot(x_gauge_1,y_gauge_1,'--','Color','k','LineWidth',1.1)
end