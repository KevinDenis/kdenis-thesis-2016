initWorkspace

selectMap = 3;
selectResTestGrid = 4; %{1:'debug';2:'coarse';3:'medium';4:'fine';'finer';'finest'};


Map_ResTestGrid = getAllComb(selectMap,selectResTestGrid);
Map_ResTestGrid=sortrows(Map_ResTestGrid,2); % sort with increasing computing time;

for ii=1:size(Map_ResTestGrid,1)
    Map_ii = Map_ResTestGrid(ii,1);
    ResTestGrid_ii = Map_ResTestGrid(ii,2);
%     LPT_Benchmark_calc(Map_ii,ResTestGrid_ii)
	LPT_Benchmark_plot(Map_ii,ResTestGrid_ii,1)
end