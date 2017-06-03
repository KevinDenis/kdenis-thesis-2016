%=========================================================================%
%                                                                         %
%              Application of LPT on Discrete Motion Planning             %
%              ----------------------------------------------             %
%                       Part 2. Build Occupancy Grid                      %
%                                                                         %
% Overview :                                                              %
%   * This code differces from BuildOccGridFromLSL because here paths     %
% length are not adjusted if they cause a collision, their are simply     %
% removed from the set (marked as blocked on the online-phase).           %
%   * This makes the creation of the StateLattice Structure simpler, since%
% calculating at which time/length/idx a certain cel is visited for the   %
% first time is not nececeaaru anymore. Just knowing which cells are      %
% occupied by taking a certain path is needed.                            %
%   * This updates the State Latice Structure.                            %
%                                                                         %
% Kevin DENIS, KU Leuven, 2016-17                                         %
% Master Thesis: Path planning algorithm for semi-autonomous              %
%                mobile robots with fast and accurate collision checking  % 
%                                                                         %
%=========================================================================%

function LSL=DMP_BuildOccGridForDMP(LSL)

gridRes=0.02;

% Get Occupancy Grid of Full and Shell Robot Import Data from bitmap picture
[XY_occ_full] = getOccXYFromBmpRobot('RobotFull_1cm.bmp',gridRes/2);
[XY_occ_shell]= getOccXYFromBmpRobot('RobotShell_1cm.bmp',gridRes/2);

% Create empty pathh occ grid
PathOccGridEmpty=robotics.BinaryOccupancyGrid(12,12,1/gridRes);
PathOccGridEmpty.GridLocationInWorld=[-6-gridRes/2 -6-gridRes/2];
PathOccGrid=copy(PathOccGridEmpty);
[ii_PathOccGrid,jj_Path]=meshgrid(1:PathOccGrid.GridSize(1),1:PathOccGrid.GridSize(2));
ii_all_Path=ii_PathOccGrid(:);
jj_all_Path=jj_Path(:);

% Moving the robot's footprint over each path
progressbar('Calculating Occupancy Grid')
n=length(LSL); 
tic
for nn=1:n
    X=LSL(nn).X;
    Y=LSL(nn).Y;
    TH=LSL(nn).TH;
    PathOccGrid=copy(PathOccGridEmpty); % just assigning doesn't work
    for idxPath=1:1:length(X)
        if idxPath==1 % use full occ grid (just once)
            XY_occ_kk=XY_occ_full;
        else % use shell grid (for the rest, faster ! but be sure that fine enough movement)
            XY_occ_kk=XY_occ_shell;
        end
        XY_occ_rot_trans=RotTransXY(XY_occ_kk ,TH(idxPath),X(idxPath),Y(idxPath));
        setOccupancy(PathOccGrid,XY_occ_rot_trans,ones(size(XY_occ_rot_trans,1),1))
    end
    occval_Path =getOccupancy(PathOccGrid,[ii_all_Path jj_all_Path], 'grid');
    ii_occ_Path=ii_all_Path(occval_Path==1);
    jj_occ_Path=jj_all_Path(occval_Path==1);
    PathOccXY=sortrows(grid2world(PathOccGrid,[ii_occ_Path jj_occ_Path]));
    LSL(nn).PathOccXY=PathOccXY;
    progressbar(nn/n)
end
toc
end