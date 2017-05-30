%=========================================================================%
%                                                                         %
%              Bézier Curve Constrained optimization Problem              %
%              ---------------------------------------------              %
%                                                                         %
% Overview :                                                              %
%   * Calculated the constrained optimization problem for                 %
%   2 point G1 fitting using a Bézier Curve                               %
%   * Returns empty path if not feasible                                  %
%   * non-convex problem due to curvature constraints and objective       %
%   function. Use of CASADI https://github.com/casadi/casadi/wiki         %
%   * A good approximated initial guess is used to speed up computation   %
%   see getInitVal for more information                                   %
%                                                                         %
% Kevin DENIS, KU Leuven, 2016-17                                         %
% Master Thesis: Path planning algorithm for semi-autonomous              %
%                mobile robots with fast and accurate collision checking  %
%                                                                         %
%=========================================================================%

function optPath=BezierCOP(initCOP)
n=initCOP.n;
P0=initCOP.P0;
P1=initCOP.P1;
x0=P0(1); y0=P0(2); th0=P0(3);
x1=P1(1); y1=P1(2); th1=P1(3);

%% Init Solver parameters
[x_init, y_init,min_val,max_val]=getInitVal(initCOP);
x=optivar(n,1,'x');
x.setInit(x_init)
x.setLb(min_val); x.setUb(max_val)

y=optivar(n,1,'y');
y.setInit(y_init)
y.setLb(min_val); y.setUb(max_val)

ipoptOptions=struct('max_iter',100,'tol',1e-3,'print_level',3);
solveOptions=struct('ipopt',ipoptOptions);
% Quick and Dirty implementation due to "unreliable" COP formulation for
% straight lines
if (IsNear(th0,th1,1e-4) && IsNear(th0,atan2(y1-y0,x1-x0),1e-4))
%     disp('this is a straight line')
    x_sol=linspace(x0,x1,n);
    y_sol=linspace(y0,y1,n);
else
%     [~,sol] = evalc('optisolve(objectiveFunction(x,y,initCOP),constraints(x,y,initCOP),solveOptions);');
    sol = optisolve(objectiveFunction(x,y,initCOP),constraints(x,y,initCOP),solveOptions);
    x_sol=optival(x);
    y_sol=optival(y);
end

[satisCons] = checkConstraints(x_sol,y_sol,initCOP);

disp(' ')
if satisCons
    t=linspace(0,1,1001)'; 
%     fprintf('Found path between: [%1.2f %1.2f %d°] and [%1.2f %1.2f %d°]\n',x0,y0,round(th0*180/pi),x1,y1,round(th1*180/pi))
    [B,dB,~,K]=BezierCurve(x_sol,y_sol,t);
    dx=dB(:,1);
    dy=dB(:,2);
    TH=atan2(dy,dx);
    S=cumtrapz(t,sqrt(dx.^2+dy.^2));
    dk=mean(K);
    intK=trapz(S,abs(K));   
    optPath=struct('x0',x0,'y0',y0,'th0',th0,'x1',x1,'y1',y1,'th1',th1,...
                   'X',B(:,1),'Y',B(:,2),'TH',TH,'S',S,'K',K, ... 
                   'k',K(1),'dk',dk,'Ltot',S(end),'intK',intK, ...
                   'PathOccXY',[], 'pathCost',intK+3*S(end), ...
                   'free',true,'idxBlocked',[],'ID',[]);
else
%     fprintf('could not find path between: [%1.2f %1.2f %d°] and [%1.2f %1.2f %d°]\n',x0,y0,round(th0*180/pi),x1,y1,round(th1*180/pi))
    optPath=[];
end
% disp(' ')
% disp('============================================')
% disp(' ')
end

function f = objectiveFunction(x,y,initCOP)
    t=initCOP.t;
    [~,~,~,kappa]=BezierCurve(x,y,t);
%     f=kappa.'*kappa;
    dk=diffMatrix(kappa);
    f=dk.'*dk;
end

function g = constraints(x,y,initCOP)

P0=initCOP.P0;
P1=initCOP.P1;

x0=P0(1); y0=P0(2); th0=P0(3);
x1=P1(1); y1=P1(2); th1=P1(3);


g{1}=[x(1) y(1) x(end) y(end)]'==[x0 y0 x1 y1]';

if abs(tan(th0))<1e-3
    g{2}= y(2) == y(1);
elseif abs(tan(th0))>1e3
    g{2} = x(2)== x(1);
else
    g{2} = y(2) == tan(th0)*(x(2)-x(1))+y(1);
end

if abs(tan(th1))<1e-3
    g{3}= y(end-1) == y(end);
elseif abs(tan(th1))>1e3
    g{3} = x(end-1) == x(end);
else
    g{3} = y(end-1) == tan(th1)*(x(end-1)-x(end))+y(end);
end

% if abs(wrap2Pi(th0)) <= pi/2
%     g{4} = x(2) >= x(1);
% else
%     g{4} = x(2) <= x(1);
% end
% 
% if abs(wrap2Pi(th1)) <= pi/2
%     g{5} = x(end-1) >= x(1);
% else
%     g{5} = x(end-1) <= x(1);
% end

% 
% t=initCOP.t;
% [~,~,~,kappa]=BezierCurve(x,y,t);
% kappa_max=initCOP.kappa_max;
% g{4}= kappa.^2 <= kappa_max^2;

end
