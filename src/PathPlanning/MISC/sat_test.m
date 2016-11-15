clear
close all
clc

rectA = [2 1; 7 1; 7 5; 2 5];
rectB = [7 1; 10 1; 10 5; 7 5]+5;

figure(); hold on
fill(rectA(:,1),rectA(:,2),'k')
fill(rectB(:,1),rectB(:,2),'g')
flag = sat (rectA, rectB)