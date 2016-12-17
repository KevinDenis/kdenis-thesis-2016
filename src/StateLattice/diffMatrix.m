function [ x_diff ] = diffMatrix( x )
%DIFFMATRIX Summary of this function goes here
%   Detailed explanation goes here
n=length(x);

DMat=-eye(n,n)+diag([ones(1,n-1)],1);
DMat(end,:)=[];

x_diff=DMat*x;


end

