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