function XYinterp=LinInterpToRes(XY,res)
%XYinterp=LinInterpToRes(XY,res)
XYinterp=[];
for ii=1:(size(XY,1)-1)
    XY_ii=XY(ii,:);
    XY_jj=XY(ii+1,:);
    dist =norm(XY_ii-XY_jj);
    n=round(dist/res)+1;
    X_interp=linspace(XY_ii(1),XY_jj(1),n).';
    Y_interp=linspace(XY_ii(2),XY_jj(2),n).';
    XYinterp=[XYinterp;X_interp Y_interp];
end
if ~all(XYinterp(end,:) == XY(end,:))
    disp('warning')
end
if ~all(XYinterp(end,:) == XY(end,:))
    disp('warning')
end
end
