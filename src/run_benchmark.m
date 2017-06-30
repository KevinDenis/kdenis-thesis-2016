initWorkspace

selectMap = [4];
selectResTestGrid = 3;

Map_ResTestGrid = getAllComb(selectMap,selectResTestGrid);
Map_ResTestGrid=sortrows(Map_ResTestGrid,2); % sort with increasing computing time;

for ii=1:size(Map_ResTestGrid,1)
    Map_ii = Map_ResTestGrid(ii,1);
    ResTestGrid_ii = Map_ResTestGrid(ii,2);
%     LPT_Benchmark_calc(Map_ii,ResTestGrid_ii)
    LPT_Benchmark_plot(Map_ii,ResTestGrid_ii)
end