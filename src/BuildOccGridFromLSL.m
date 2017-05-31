%=========================================================================%
%                                                                         %
%                          Build Occupancy Grid                           %
%                          --------------------                           %
%                        (Path and Obstacle Based)                        %
%                                                                         %
% Calculates the occupancy of the wheelchair going over each set of paths %
% of the Local State Lattice. This can take up to 5min.                   %
% Path based lookup table will be converted to obstacle based lookup table%
% occupancy grid (quite efficiently, ~10sec)                              % 
%                                                                         %
% Overview :                                                              %
%   * First, the traditional "Path Based" lookup table is calculated, by  %
% letting the shape of the robot move over each paths and storing each    %
% time a grid is visited for the first time along with the                %
% "path Idx"/"time"/"length". This, to know what space the robot occupies %
% atevery pose along the path                                             %
% calculated in the previous step. This part updates the LSL Structure    %
%                                                                         %
%   * Secondly, the Obstacke Based Occupancy grid is generated using the  %
% previously calculated data.  A clever trick is used in order to generate%
% the obstacle based lookup table in an efficient way.                    %
% Clever trick explained below, in "Obstacle Based Approach".             %
% This part of the code will create a new structure, ObstacleTable.       %
% A Matrix form of this structure will also be kept, XY_ObsTable          %
% ObstacleTable contains all paths where the robot occupies a certain     %
% certain cell, and at which position along that path ("time"/"idx')      %
% this happens.                                                           %
%                                                                         %
% Kevin DENIS, KU Leuven, 2016-17                                         %
% Master Thesis: Path planning algorithm for semi-autonomous              %
%                mobile robots with fast and accurate collision checking  % 
%                                                                         %
%=========================================================================%

%=========================================================================%
%                                                                         %
%                 Local State Lattice Structure (LSL)                     %
%                 -----------------------------------                     %
%             [ x0 y0 th0 x1 y1 th1 X Y TH S K k dk Ltot                  %
%              intK PathOccXY pathCost free idxBlocked ID]                %
%                                                                         %
%                        Obstacle Table Structure                         %
%                        ------------------------                         %
%                     [ x y (path)ID (path)blockedIdx]                    %
%                                                                         %
%=========================================================================%

function [StateLattice,ObstacleTable,XY_ObsTable]=BuildOccGridFromLSL(StateLattice)
tic
gridRes=0.02;
% Get Occupancy Grid of Full and Shell Robot
[XY_occ_full] = getOccXYFromBmpRobot('RobotFull_1cm.bmp',gridRes/2);
[XY_occ_shell]= getOccXYFromBmpRobot('RobotShell_1cm.bmp',gridRes/2);

%% Path based approach
progressbar('Calculating Occupancy Grid [Path Based]')

n=length(StateLattice); 
ObsTable=zeros(1,4);
idxTable=0;
for nn=1:n
%     tic
    X=StateLattice(nn).X;
    Y=StateLattice(nn).Y;
    TH=StateLattice(nn).TH;
    PathOccXY=zeros(1,3);
    idxOccXY=0;
    for idxPath=1:1:length(X)
        if idxPath==1 % use full occ grid (just once)
            XY_occ_kk=XY_occ_full;
        else % use shell grid (for the rest, faster ! but be sure that fine enough movement)
            XY_occ_kk=XY_occ_shell;
        end
        
        XY_occ_rot_trans=RotTransXY(XY_occ_kk ,TH(idxPath),X(idxPath),Y(idxPath));
        PathOccXY_kk = RoundToRes(XY_occ_rot_trans,gridRes);
        idxOccXY=idxOccXY(end)+(1:length(PathOccXY_kk));
        PathOccXY(idxOccXY,:)=[PathOccXY_kk repelem(idxPath,size(PathOccXY_kk,1),1)];
        
    end
    [~,idxUnique,~] =unique(PathOccXY(:,1:2),'rows','stable'); % keep unique x-y set, but keep sorted by "time" (-stable option) (3rd colomn)
    PathOccXY=PathOccXY(idxUnique,:);
    
    idxTable=idxTable(end)+(1:size(PathOccXY,1));
    ObsTable(idxTable,[1 2 4 3])=[PathOccXY repelem(nn,length(idxTable),1)];   % [x y (path)ID blockedIdx]
    
    StateLattice(nn).PathOccXY=PathOccXY;
    progressbar(nn/n)
%     toc
end
disp(['Calculation of Path Based Occupancy Grid took ', num2str(toc,5),' sec'])

%% Obstacle Based Approach
tic
XY_ObsTable=ObsTable(:,1:2);
[XY_ObsTable_S,IdxSorted]=sortrows(XY_ObsTable); % sorted is needed for the loop (see later)
ObsTable_S=ObsTable(IdxSorted,:);

[XY_ObsTable_U,idxUnique,~] = unique(XY_ObsTable_S,'rows','stable'); 
% stable not needed since allready sorted beforehad, 
% but still used to be sure unique doesn't change the row sequence
% idxUnique holds all the unique rows idx of XY_Table_S.
% Thus it hols all unique XY paires.

% Clever trick :
% --------------
% By comparing the unique idx set of x-y paires way ALL row indicand the sorted matrix of x-y
% paires, one can select in a very fast es that have to be
% "stacked" in one, in order to get all path ID's corresponding to one x-y
% poisition and their corresponding index at what time this happens !
% This avoides the use of findrow_mex, which is rather slow.
% Simple example : XY_sorted = [0 0; 0 1; 1 1; 1 1; 1 2]
%                  idxUnique = [1 2 3 5];
%                                   I can "merge" 3:4.

n=length(idxUnique); 
ObstacleTable(1,1)=struct('x',[],'y',[],'ID',[],'Idx',[]); % Empty clean ObstacleTable, since it is not cleaned by initWorkspace
ObstacleTable(1:n,1)=struct('x',[],'y',[],'ID',[],'Idx',[]);
progressbar('Calculating Occupancy Grid [Obstacle Based]')
for ii=1:n
    ObstacleTable(ii).x=XY_ObsTable_U(ii,1);
    ObstacleTable(ii).y=XY_ObsTable_U(ii,2);
    if ii~=length(idxUnique)
        idxPath=idxUnique(ii):idxUnique(ii+1)-1; % take all the non-unique idx intil the next unique idx.
    else
        idxPath=idxUnique(ii):length(IdxSorted); % don't use idxSorted(end), because this end refers to the unsorted array !
    end
    ID_ii=ObsTable_S(idxPath,3);
    Idx_ii=ObsTable_S(idxPath,4);
    [ID_ii,idxSortID]=sortrows(ID_ii); % this is actually not needed, but just for estetic reasons (start with smallest path ID)
    Idx_ii=Idx_ii(idxSortID); % corresponding idx have to be sort in the same manner as the ID
    ObstacleTable(ii).ID=ID_ii;
    ObstacleTable(ii).Idx=Idx_ii;
    progressbar(ii/n)
end
XY_ObsTable = XY_ObsTable_U; % Keep XY_ObsTable to perform fast matrix searches. It has the same x y idx pair as ObstacleTable (very important !)
disp(['Calculation of Obstacle Based Occupancy Grid took ', num2str(toc,5),' sec'])