% initWorkspace
clc
close all


if ~exist('LSL', 'var'); load('LSL_cloth.mat'); end
if ~exist('ObstacleTable', 'var'); load('ObstacleTable_cloth.mat');  end
if ~exist('XY_ObsTable', 'var'); load('XY_ObsTable_cloth.mat'); end

%{
XY_ObsTable_red_00=XY_ObsTable;
XY_ObsTable_red_10=XY_ObsTable;
XY_ObsTable_red_20=XY_ObsTable;
XY_ObsTable_red_30=XY_ObsTable;
XY_ObsTable_red_40=XY_ObsTable;
XY_ObsTable_red_50=XY_ObsTable;
XY_ObsTable_red_60=XY_ObsTable;
XY_ObsTable_red_70=XY_ObsTable;
XY_ObsTable_red_80=XY_ObsTable;
XY_ObsTable_red_90=XY_ObsTable;

[LSL_red_00,ObstacleTable_red_00]=reducePathDensityFromLSL(LSL,ObstacleTable,0.0);
[LSL_red_10,ObstacleTable_red_10]=reducePathDensityFromLSL(LSL,ObstacleTable,0.1);
[LSL_red_20,ObstacleTable_red_20]=reducePathDensityFromLSL(LSL,ObstacleTable,0.2);
[LSL_red_30,ObstacleTable_red_30]=reducePathDensityFromLSL(LSL,ObstacleTable,0.3);
[LSL_red_40,ObstacleTable_red_40]=reducePathDensityFromLSL(LSL,ObstacleTable,0.4);
[LSL_red_50,ObstacleTable_red_50]=reducePathDensityFromLSL(LSL,ObstacleTable,0.5);
[LSL_red_60,ObstacleTable_red_60]=reducePathDensityFromLSL(LSL,ObstacleTable,0.6);
[LSL_red_70,ObstacleTable_red_70]=reducePathDensityFromLSL(LSL,ObstacleTable,0.7);
[LSL_red_80,ObstacleTable_red_80]=reducePathDensityFromLSL(LSL,ObstacleTable,0.8);
[LSL_red_90,ObstacleTable_red_90]=reducePathDensityFromLSL(LSL,ObstacleTable,0.9);
%}

n_red_00=length(ObstacleTable_red_00);
n_red_10=length(ObstacleTable_red_10);
n_red_20=length(ObstacleTable_red_20);
n_red_30=length(ObstacleTable_red_30);
n_red_40=length(ObstacleTable_red_40);
n_red_50=length(ObstacleTable_red_50);
n_red_60=length(ObstacleTable_red_60);
n_red_70=length(ObstacleTable_red_70);
n_red_80=length(ObstacleTable_red_80);
n_red_90=length(ObstacleTable_red_90);

stepSize=10;

n=min([n_red_00,n_red_10,n_red_20,n_red_40,n_red_50,n_red_60,n_red_70,n_red_80,n_red_90]);
n=floor(n/stepSize);

dt_red_00=zeros(n,1);
dt_red_10=zeros(n,1);
dt_red_20=zeros(n,1);
dt_red_30=zeros(n,1);
dt_red_40=zeros(n,1);
dt_red_50=zeros(n,1);
dt_red_60=zeros(n,1);
dt_red_70=zeros(n,1);
dt_red_80=zeros(n,1);
dt_red_90=zeros(n,1);
ppm = ParforProgressStarter2('Calculating...', n, 1, 0, 1, 1);
for ii=1:n
    jj=ii*stepSize;
    
    XY_obs_red_00=XY_ObsTable_red_00(jj,:);
    XY_obs_red_10=XY_ObsTable_red_10(jj,:);
    XY_obs_red_20=XY_ObsTable_red_20(jj,:);
    XY_obs_red_30=XY_ObsTable_red_30(jj,:);
    XY_obs_red_40=XY_ObsTable_red_40(jj,:);
    XY_obs_red_50=XY_ObsTable_red_50(jj,:);
    XY_obs_red_60=XY_ObsTable_red_60(jj,:);
    XY_obs_red_70=XY_ObsTable_red_70(jj,:);
    XY_obs_red_80=XY_ObsTable_red_80(jj,:);
    XY_obs_red_90=XY_ObsTable_red_90(jj,:);
    
    [dt_red_00(ii)]=UpdateStateLattice(XY_obs_red_00,LSL_red_00,ObstacleTable_red_00,XY_ObsTable_red_00);
    [dt_red_10(ii)]=UpdateStateLattice(XY_obs_red_10,LSL_red_10,ObstacleTable_red_10,XY_ObsTable_red_10);
    [dt_red_20(ii)]=UpdateStateLattice(XY_obs_red_20,LSL_red_20,ObstacleTable_red_20,XY_ObsTable_red_20);
    [dt_red_30(ii)]=UpdateStateLattice(XY_obs_red_30,LSL_red_30,ObstacleTable_red_30,XY_ObsTable_red_30);
    [dt_red_40(ii)]=UpdateStateLattice(XY_obs_red_40,LSL_red_40,ObstacleTable_red_40,XY_ObsTable_red_40);
    [dt_red_50(ii)]=UpdateStateLattice(XY_obs_red_50,LSL_red_50,ObstacleTable_red_50,XY_ObsTable_red_50);
    [dt_red_60(ii)]=UpdateStateLattice(XY_obs_red_60,LSL_red_60,ObstacleTable_red_60,XY_ObsTable_red_60);
    [dt_red_70(ii)]=UpdateStateLattice(XY_obs_red_70,LSL_red_70,ObstacleTable_red_70,XY_ObsTable_red_70);
    [dt_red_80(ii)]=UpdateStateLattice(XY_obs_red_80,LSL_red_80,ObstacleTable_red_80,XY_ObsTable_red_80);
    [dt_red_90(ii)]=UpdateStateLattice(XY_obs_red_90,LSL_red_90,ObstacleTable_red_90,XY_ObsTable_red_90);
    
    ppm.increment(ii);
end

delete(ppm);

[dt_red_00,~]=sort(dt_red_00);
dt_red_00(end-stepSize:end)=[];

[dt_red_10,~]=sort(dt_red_10);
dt_red_10(end-stepSize:end)=[];

[dt_red_20,~]=sort(dt_red_20);
dt_red_20(end-stepSize:end)=[];

[dt_red_30,~]=sort(dt_red_30);
dt_red_30(end-stepSize:end)=[];

[dt_red_40,~]=sort(dt_red_40);
dt_red_40(end-stepSize:end)=[];

[dt_red_50,~]=sort(dt_red_50);
dt_red_50(end-stepSize:end)=[];

[dt_red_60,~]=sort(dt_red_60);
dt_red_60(end-stepSize:end)=[];

[dt_red_70,~]=sort(dt_red_70);
dt_red_70(end-stepSize:end)=[];

[dt_red_80,~]=sort(dt_red_80);
dt_red_80(end-stepSize:end)=[];

[dt_red_90,~]=sort(dt_red_90);
dt_red_90(end-stepSize:end)=[];


figure()
hold on
histogram(dt_red_00);
histogram(dt_red_10);
histogram(dt_red_20);
histogram(dt_red_30);
histogram(dt_red_40);
histogram(dt_red_50);
histogram(dt_red_60);
histogram(dt_red_70);
histogram(dt_red_80);
histogram(dt_red_90);
legend('00','10','20','30','40','50','60','70','80','90')

figure()
hold on
histfit(dt_red_00);
histfit(dt_red_10);
histfit(dt_red_20);
histfit(dt_red_30);
histfit(dt_red_40);
histfit(dt_red_50);
histfit(dt_red_60);
histfit(dt_red_70);
histfit(dt_red_80);
histfit(dt_red_90);
legend('00','10','20','30','40','50','60','70','80','90')

pd_00 = fitdist(dt_red_00,'normal');
pd_10 = fitdist(dt_red_10,'normal');
pd_20 = fitdist(dt_red_20,'normal');
pd_30 = fitdist(dt_red_30,'normal');
pd_40 = fitdist(dt_red_40,'normal');
pd_50 = fitdist(dt_red_50,'normal');
pd_60 = fitdist(dt_red_60,'normal');
pd_70 = fitdist(dt_red_70,'normal');
pd_80 = fitdist(dt_red_80,'normal');
pd_90 = fitdist(dt_red_90,'normal');


x = [length(LSL_red_00) length(LSL_red_10) length(LSL_red_20) length(LSL_red_30) ...
    length(LSL_red_40) length(LSL_red_50) length(LSL_red_60) length(LSL_red_70) ...
    length(LSL_red_80) length(LSL_red_90)];
y = [pd_00.mu pd_10.mu pd_20.mu pd_30.mu pd_40.mu pd_50.mu pd_60.mu pd_70.mu pd_80.mu pd_90.mu];
err = [pd_00.sigma pd_10.sigma pd_20.sigma pd_30.sigma pd_40.sigma pd_50.sigma pd_60.sigma pd_70.sigma pd_80.sigma pd_90.sigma];

figure()
errorbar(x,y,err)

figure()
logfit(x,y)

% figure()
% hold on
% plot(n_paths_cloth,dt_normal,'.');
% plot(n_paths_circ,dt_circ,'.');

%% Functions

function [LSL_red,ObstacleTable_red]=reducePathDensityFromLSL(LSL,ObstacleTable,redFactor)

pathsToDelete = floor(redFactor*length(LSL));

deletePathID=unique(round(linspace(1,length(LSL),pathsToDelete)));

LSL_red = LSL;
LSL_red(deletePathID)=[];

ObstacleTable_red = ObstacleTable;

for ii=1:size(ObstacleTable,1)
    ObstacleTable_ID_ii=ObstacleTable(ii).ID;
    
    idxToRemove=(ismember(ObstacleTable_ID_ii,deletePathID));
    ObstacleTable_ID_ii(idxToRemove)=[];
    ObstacleTable_red(ii).ID=ObstacleTable_ID_ii;
end
end



function [dt,n_paths]=UpdateStateLattice(XY_ObsGrid,LSL,ObstacleTable,XY_Table)
xy_obs_end=XY_ObsGrid(end,:);
tic
% idxRow=find(ismember(XY_Table,XY_ObsGrid,'rows'));
idxRow=findrow_mex(XY_Table,xy_obs_end);
IDOccPaths = [ObstacleTable(idxRow).ID];
IdxOccPaths = [ObstacleTable(idxRow).Idx];
for jj=1:length(IDOccPaths)
    LSL(IDOccPaths(jj)).free=false;
    if isempty(LSL(IDOccPaths(jj)).idxBlocked) || LSL(IDOccPaths(jj)).idxBlocked > IdxOccPaths(jj)
        LSL(IDOccPaths(jj)).idxBlocked=IdxOccPaths(jj);
    end
end
dt=toc;
n_paths=length(IDOccPaths);
end
%}