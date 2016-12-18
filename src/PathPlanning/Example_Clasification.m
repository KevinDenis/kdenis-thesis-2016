% clear 
close all
clc


x=ginput(3);
y=ginput(3);

t = optivar();
a = optivar(2);
a.setInit([1;1])
b = optivar();
b.setInit([0.5])



nlp = optisolve(-t,{x*a-b<=-t, y*a-b>=t,norm_2(a)==1,t>0});
a=optival(a);
b=optival(b);
t=optival(t);
sepLineX=0:0.1:1;
sepLineY=(b-a(1)*sepLineX)/a(2);

figure()
hold on
scatter(x(:,1),x(:,2))
scatter(y(:,1),y(:,2))
quiver(sepLineX(6),sepLineY(6),a(1)/3,a(2)/3)
plot(sepLineX,sepLineY)
axis equal