clear 
close all
clc

x1=0; 
y1=0; 
th1=pi/4; 
Pstart=[x1,y1,th1]; 
xend=0; 
yend=2; 
thend=pi/4; 
Pend=[xend,yend,thend]; 
n=4; 


initCOP=struct('Pstart',Pstart,'Pend',Pend,'n',n);

getInitValX(initCOP)
getInitValY(initCOP)

scatter(getInitValX(initCOP),getInitValY(initCOP))