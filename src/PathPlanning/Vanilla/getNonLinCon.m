function [C,Ceq]=getNonLinCon(x,y,t,r,obs)
[B,~,~,kappa]=BezierCurve([x,y],t);

C=[norm(kappa,inf)-5];

if ~isempty(obs)
    k = dsearchn(obs,B);
    robot_k=B;
    obs_k=obs(k,:);
    a=obs_k-robot_k;
    b=(Norm2(obs_k).^2-Norm2(robot_k).^2)/2;
    hyperplane_obs=zeros(size(t));
    for ii=1:length(t)
        hyperplane_obs(ii)=a(ii,:)*obs_k(ii,:)';
    end
    hyperplane_rob=zeros(size(t));
    for ii=1:length(t)
        hyperplane_rob(ii)=a(ii,:)*robot_k(ii,:)';
    end
    C=[C;hyperplane_rob-b-0.01];
end

Ceq=[];
end
