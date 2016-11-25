clear
% close all
clc
clf

x=-1:0.01:1.5;
r_veh=0.5;
r=[0 0]';
ang=0:0.1:2*pi+0.1; 
rx=r(1)+r_veh*cos(ang);
ry=r(2)+r_veh*sin(ang);


r=[rx' ry'];
obs=[0.5 0.5;
       1.0 0.5;
       1.0 1.0;
       0.5 1.0];
obs_filled=[];
for ii=1:length(obs)-1
    obs_filled=[obs_filled;fillline(obs(ii,:),obs(ii+1,:),10)];
end
obs_filled=[obs_filled;fillline(obs(end,:),obs(1,:),10)];
obs=obs_filled;
   
[k,d] = dsearchn(obs,r);
[d,l]=min(d);
obs_i= obs(k(l),:)';
r_i=r(l,:)';

a= obs_i-r_i;
b=(obs_i'*obs_i-r_i'*r_i)/2;

y=(b-a(1)*x)/a(2);
a'*obs_i-b>=0
a'*r_i-b<=0




figure(1)
hold on
plot(rx,ry);
fill(obs(:,1),obs(:,2),'k')
scatter(obs_i(1), obs_i(2),'filled','r')
scatter(r_i(1), r_i(2),'filled','MarkerFaceColor',[0 0.4470 0.7410])

plot(x,y,'color',[0.4,0.4,0.4])
axis equal