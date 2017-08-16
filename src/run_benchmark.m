initWorkspace

%% user settings
benchmark_type = 1; % 0: Path Planner Performance 1: Time Performances
saveFigure = 0;
% Path Planning Performance user settings
PPP_selectMap = 3;
PPP_selectResTestGrid = 4; %{1:'debug';2:'coarse';3:'medium';4:'fine';'finer';'finest'};
PPP_usePrecomputedData = 1;
% Time Performances
TP_selectTest = 1; % 0: single occupied cell 1: simulated environment
stepSize_SingleCell = 20; % one every stepSize_Single is simulated to be occupied
usePrecomputed_MultiCell = 1; % use precomputed data for the simulated environment
saveComputedData_MultiCell = 0; % save computed data for the simulated environment

switch benchmark_type
    case 0
        Map_ResTestGrid = getAllComb(PPP_selectMap,PPP_selectResTestGrid);
        Map_ResTestGrid=sortrows(Map_ResTestGrid,2); % sort with increasing computing time;

        for ii=1:size(Map_ResTestGrid,1)
            Map_ii = Map_ResTestGrid(ii,1);
            ResTestGrid_ii = Map_ResTestGrid(ii,2);
            if ~PPP_usePrecomputedData; LPT_PPP_Benchmark_calc(Map_ii,ResTestGrid_ii); end
            LPT_PPP_Benchmark_plot(Map_ii,ResTestGrid_ii,saveFigure)
        end
    case 1
        switch TP_selectTest
            case 0
                LPT_TP_Benchmark_singleObs(stepSize_SingleCell,saveFigure)
            case 1
                LPT_TP_Benchmark_multiObs(usePrecomputed_MultiCell,saveComputedData_MultiCell,saveFigure)
        end
end