function [bezierMatrix]=getBezierMatrix(n)

syms t

bM=bernsteinMatrix(n, t);

bezierMatrix=zeros(n+1);

for ii=1:length(bM)
    bezierMatrix(ii,:)=sym2poly(bM(ii));
end

bezierMatrixString=['['];

for ii=1:length(bM)
    for jj=1:length(bM)
        bezierMatrixString=[bezierMatrixString,num2str(bezierMatrix(ii,jj))];
        if jj ~= length(bM)
            bezierMatrixString=[bezierMatrixString,','];
        end
    end
    if ii ~= length(bM)
        bezierMatrixString=[bezierMatrixString,';'];
    end
end

bezierMatrixString=[bezierMatrixString,']'];

disp(bezierMatrixString)


end