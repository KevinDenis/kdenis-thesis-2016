function optPath=BezierCOP(initCOP)
t=initCOP.t;
n=initCOP.n;
Pstart=initCOP.Pstart;
Pend=initCOP.Pend;
x1=Pstart(1); y1=Pstart(2); th1=Pstart(3);
xend=Pend(1); yend=Pend(2); thend=Pend(3);

max_val=max([x1 y1 xend yend])*2;
min_val=-max_val;

%% Solver
[x_init, y_init]=getInitVal(initCOP);
x=optivar(n,1,'x');
x.setInit(x_init)
x.setLb(min_val); x.setUb(max_val)

y=optivar(n,1,'y');
y.setInit(y_init)
y.setLb(min_val); y.setUb(max_val)

ipoptOptions=struct('max_iter',300,'tol',1e-3,'print_level',3);
solveOptions=struct('ipopt',ipoptOptions);

sol = optisolve(objectiveFunction(x,y,initCOP),constraints(x,y,initCOP),solveOptions);
x_sol=optival(x);
y_sol=optival(y);

[satisCons] = checkConstraints(x_sol,y_sol,initCOP);

disp(' ')
if satisCons
    fprintf('Found path between: [0 0 %d°] and [%1.2f %1.2f %d°]\n',round(th1*180/pi),xend,yend,round(thend*180/pi))
    [~,dB,~,kappa]=BezierCurve(x_sol,y_sol,t);
    dx=dB(:,1);
    dy=dB(:,2);
    s=trapz(t,sqrt(dx.^2+dy.^2));
    intKappaT=trapz(t,kappa.^2);   
    optPath=struct('th1',th1,'xend',xend,'yend',yend,'thend',thend,'x',x_sol,'y',y_sol,'s',s,'intKappaT',intKappaT);
else
    fprintf('could not find path between: [0 0 %d°] and [%1.2f %1.2f %d°]\n',round(th1*180/pi),xend,yend,round(thend*180/pi))
    optPath=[];
end
disp(' ')
disp('============================================')
disp(' ')
end



