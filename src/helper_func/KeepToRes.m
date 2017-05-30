function XYkeep=KeepToRes(XY,res)
%XYkeep=KeepToRes(XY,res)
XYkeep=XY(1,:);
cumulDist=0;
for ii=2:(size(XY,1)-1)
    XY_ii=XY(ii-1,:);
    XY_jj=XY(ii,:);
    dist =norm(XY_ii-XY_jj);
    cumulDist=cumulDist+dist;
    if cumulDist>res
        XYkeep=[XYkeep;XY_jj];
        cumulDist=0;
    end
end
XYkeep=[XYkeep;XY(end,:)];
end