
[k,d] = dsearchnCasadi(obs,P);
C=zeros(length(k),1);
C_temp=zeros(1,size(obs,1));
robot_k=B;
obs_k=obs(k,:);
% a=obs_k-robot_k;
for ii=1:length(k)
    C_temp(1)=(obs_k(ii,:)-robot_k(ii,:))*(obs_k(1,:)-(obs_k(ii,:)+robot_k(ii,:))/2)'-.01;
    C_temp(2)=(obs_k(ii,:)-robot_k(ii,:))*(obs_k(2,:)-(obs_k(ii,:)+robot_k(ii,:))/2)'-.01;
    C_temp(3)=(obs_k(ii,:)-robot_k(ii,:))*(obs_k(3,:)-(obs_k(ii,:)+robot_k(ii,:))/2)'-.01;
    C_temp(4)=(obs_k(ii,:)-robot_k(ii,:))*(obs_k(4,:)-(obs_k(ii,:)+robot_k(ii,:))/2)'-.01;
    C(ii)=min(C_temp);    
end

crash=find(C<0);

function [k,d] = dsearchnCasadi(obs,P) % performs the search without using a triangulation.
% P : path [x_i y_i]
% obs : obstacle [x_i y_i]
import casadi.*

k = MX.sym('k',size(P,1),1); % index
d = MX.sym('d',size(P,1),1); % length

for i = 1:size(P,1) 
    Pi_rep = repmat(P(i,:),size(obs,1),1);
    [d(i),k(i)] = minCasadi(sum((obs-Pi_rep).^2,2));
end
end

function [minVal,idx] = minCasadi(x)
%MINCASADI Summary of this function goes here
%   Detailed explanation goes here
minVal = x(1);
for i=2:length(x)
    minVal = min(minVal, x(i));
end
idx = find(x == minVal);
idx=idx(1);
end


