clear
close all
clc
co=get(groot,'DefaultAxesColorOrder');
lw=2;


% f = @(alpha,h,d) 1-alpha*exp(-1/(2*h.^2)*d.^2);

d = 0:0.001:50;

alpha1=0.99;
h1=24;
h1Factor=-(2*h1.^2)^-1;
psi1 = 1-alpha1*exp(h1Factor*d.^2);

alpha2=0.8;
h2=24;
psi2 = 1-alpha2*exp(-1/(2*h2^2)*d.^2);

alpha3=0.99;
h3=30;
psi3 = 1-alpha3*exp(-1/(2*h3^2)*d.^2);


% fig=figureFullScreen(1);
figure(1)
hold on
plot(d,psi1,'Color',co(1,:),'LineWidth',lw)
plot(d,psi2,'Color',co(2,:),'LineWidth',lw)
plot(d,psi3,'Color',co(5,:),'LineWidth',lw)
hold off
set(gca,'FontSize',14)
xlabel('$\vert f_{\tau}^{(i)}-f_{\tau}^{(j)}\vert$ [pixel]','Interpreter','LaTeX','FontSize',16)
ylabel('$\psi(\vert f_{\tau}^{(i)}-f_{\tau}^{(j)}\vert)$','Interpreter','LaTeX','FontSize',16)
l=legend('\alpha = 0.99, h=24','\alpha = 0.80, h=24','\alpha = 0.99, h=30','Location','SE');
set(l,'FontSize',14);

set(gca, 'box', 'off')

saveCurrentFigure('InteractionPotentialFunction')

% figure(1)
% for alpha = 0.2:0.2:1
% plot(d,f(alpha,h,d))
% hold on
% end
% 
% alpha = 0.95;
% 
% figure(2)
% for h = 10:2:30
% plot(d,f(alpha,h,d))
% hold on
% end