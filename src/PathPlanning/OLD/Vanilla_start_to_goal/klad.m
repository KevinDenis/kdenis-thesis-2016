% function [ x_diff ] = diffMatrix( x )
%DIFFMATRIX Summary of this function goes here
%   Detailed explanation goes here
n=2*4

A=eye(n+1,n+1)-diag([ones(1,n-1)],2)
A([end-1,end],:)=[]




% end