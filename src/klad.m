A=XY_ObsTable_circ;
B=XY_ObsTable_circ(end,:);

hash = randn(size(A,2),1);

b=B*hash;
a=A*hash;

tic
idx=find(ismember(A,B,'rows'))
toc

tic
idx=find(ismember(a,b))
toc