clear
close all
clc
n=2;
syms t z

bM=bernsteinMatrix(n,t);

bezierMatrix=zeros(n+1);

for ii=1:length(bM)
    bezierMatrix(ii,:)=sym2poly(bM(ii));
end

bernsteinMatrix(2,(z-1))