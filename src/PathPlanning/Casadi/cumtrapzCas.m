function z = cumtrapzCas(x,y)
import casadi.*

[m,n]=size(x);
z= MX.sym('z',m,n);
z(1)=x(1)*0;
for ii=2:length(x)  
    z(ii)=trapzCas(x(1:ii),y(1:ii));
end
end
