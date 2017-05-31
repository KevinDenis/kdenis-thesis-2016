function [ XY_Round ] = RoundToRes(XY,res)
%[ XY_Round ] = RoundToRes(XY,res)
XY_Round=round(XY./res).*res;
XY_Round=round(XY_Round*100)./100;
% XY_Round=unique(XY_Round,'rows'); 
% a second rounding operation is needed due to small numerical errors
% during the multiplication of the previous step.
% Alltough unique is a costly operation, this will speed up the dynamical
% matrix allocation, because there are less elements.
end

