initPlotScripts

if ~exist('LSL_cloth', 'var'); load('LSL_cloth.mat'); LSL_cloth = LSL; end
if ~exist('ObstacleTable_cloth', 'var'); load('ObstacleTable_cloth.mat'); ObstacleTable_cloth=ObstacleTable; end
if ~exist('LSL_circ', 'var'); load('LSL_circ.mat'); LSL_circ = LSL; end
if ~exist('ObstacleTable_circ', 'var'); load('ObstacleTable_circ.mat'); ObstacleTable_circ=ObstacleTable; end



n_cloth = length(ObstacleTable_cloth);
paths_per_cell_cloth=zeros(n_cloth,1);
for ii = 1:n_cloth
    paths_per_cell_cloth(ii)=length(ObstacleTable_cloth(ii).ID);
end

n_circ = length(ObstacleTable_circ);
paths_per_cell_circ=zeros(n_circ,1);
for ii = 1:n_circ
    paths_per_cell_circ(ii)=length(ObstacleTable_circ(ii).ID);
end

figure(1)
histogram(paths_per_cell_cloth)

figure(2)
histogram(paths_per_cell_circ)
