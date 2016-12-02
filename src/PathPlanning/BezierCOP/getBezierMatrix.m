function [bezierMatrix,zeroToZMatrix,zToTMatrix]=getBezierMatrix(n,z_val)

syms t z

bM=bernsteinMatrix(n,t);

bezierMatrix=zeros(n+1);

for ii=1:length(bM)
    bezierMatrix(ii,:)=sym2poly(bM(ii));
end
% if isnumeric(z)
%     zDiagBegin=zeros(1,n+1);
%     zDiagEnd=zeros(1,n+1);
% end
% for ii=1:n+1
%     zDiagBegin(ii)=z^(ii-1);
%     zDiagEnd(ii)=(1-z)^(ii-1);
% end
% zDiagBegin=flip(zDiagBegin);
% zDiagEnd=flip(zDiagEnd);
% zMatrixBegin=diag(zDiagBegin);
% zMatrixEnd=diag(zDiagEnd);
% zeroToZMatrix=bezierMatrix\zMatrixBegin*bezierMatrix;
% zToTMatrix=bezierMatrix\zMatrixEnd*bezierMatrix;

zeroToZMatrix=sym(zeros(n+1));
for ii=0:n
    zeroToZ_vector_full=sym(zeros(1,n+1));
    zeroToZ_vector=flip(bernsteinMatrix(ii,(1-z)));
    zeroToZ_vector_full(1:length(zeroToZ_vector))=zeroToZ_vector;
    zeroToZMatrix(ii+1,:)=zeroToZ_vector_full;
end


zToTMatrix=sym(zeros(n+1));
for ii=n:-1:0
    zToT_vector_full=sym(zeros(1,n+1));
    zToT_vector=bernsteinMatrix(ii,(1-z));
    zToT_vector_full(1:length(zToT_vector))=zToT_vector;
    zToTMatrix(ii+1,:)=zToT_vector_full;
end
zToTMatrix=fliplr(flipud(zToTMatrix));

if isnumeric(z_val)
    zeroToZMatrix=double(subs(zeroToZMatrix,z,z_val));
    zToTMatrix=double(subs(zToTMatrix,z,z_val));
end

end