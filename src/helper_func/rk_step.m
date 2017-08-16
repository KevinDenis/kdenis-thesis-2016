function xnext = rk_step(ode,x,u,h) %Runge-Kutta integrator
    k1 = ode(x,u);
    k2 = ode(x+h/2.0*k1,u);
    k3 = ode(x+h/2.0*k2,u);
    k4 = ode(x+h*k3,u);
    xnext = x+h/6.0*(k1+2*k2+2*k3+k4);
end
