function [Path_Cleaned] = CleanupLSL(Path)
%[Path_Cleaned] = CleanupLSL(Path)
%   This function will cleanup the set of paths by only keeping paths with 
%   a unique start-destination
%   will also remove paths which are irrelevant 
%   paths wich are a subset of other paths, starting and leading to a pose
%   with is allready been achieved previously
%   
%   This could be done in a more strickt and regourus way, by only keeping
%   the longest paths. E.g. 1D example : 
%   Path 1 : 0    --> 0.25
%   Path 2 : 0.25 --> 0.5
%   Path 3 : 0    --> 0.5
%   Only keep path 3
%   Slow implementation atm, but is not beeing used.

vertices=[[Path.x0].',[Path.y0].',[Path.th0].' [Path.x1].',[Path.y1].',[Path.th1].'];
[~,idx,~]=unique(vertices,'rows','stable');
Path=Path(idx);
vertices=vertices(idx,:);


%% 
toDelete=zeros(1);
nn=1;
for ii=2:size(vertices,1)
    % gather all voxels before current voxel
    prevVerticesSet=vertices(1:ii-1,:); 
    prevStartVerticesSet=vertices(1:ii-1,1:3);
    prevEndVerticesSet=vertices(1:ii-1,4:6);
    % gather current voxel
    vertices_ii=vertices(ii,:);    
    startVoxel_ii=vertices_ii(1:3);
    endVoxel_ii=vertices_ii(4:6);
    
    idxEndPrevVox=findVectorRev(prevEndVerticesSet,endVoxel_ii);
    if ~isnan(idxEndPrevVox)
        prevEndVoxels_start=prevStartVerticesSet(idxEndPrevVox,:);
        idxStartOldToStartNew=findVector(prevVerticesSet,[prevEndVoxels_start startVoxel_ii]);
        if ~isnan(idxStartOldToStartNew)
            toDelete(nn)=ii;
            nn=nn+1;
        end
    end
end

Path_Cleaned=Path;
if (toDelete(1) ~= 0); Path_Cleaned(toDelete)=[]; end

% assign unique path ID
for ii=1:length(Path_Cleaned)
    Path_Cleaned(ii).ID=ii;
end
end

%% NOT IMPLEMENTED JET
function keepLongestPaths()
StateLattice_fwd = getForwardMotionFromStateLattice(StateLattice);
StateLattice_fwd=removeTurnOnTheSpot(StateLattice_fwd);

dk_all=[StateLattice_fwd.dk];


tic
idxRemove=zeros(1);
nn=1;
for ii=1:length(StateLattice_fwd)
    dk_ii=StateLattice_fwd(ii).dk;
    X_ii=StateLattice_fwd(ii).X;
    Y_ii=StateLattice_fwd(ii).Y;
    idxDK=find(ismembertol(dk_all,dk_ii,1e-6)==1);
    for jj=1:length(idxDK)
        X_jj=[StateLattice_fwd(idxDK(jj)).X];
        if ~IsNear(X_jj(1),X_jj(end),1e-3)
            Y_jj=[StateLattice_fwd(idxDK(jj)).Y];
            Y2 = interp1(X_jj,Y_jj,X_ii,'linear',inf);
            distError = Y_ii-Y2;
        else
            Y_jj=[StateLattice_fwd(idxDK(jj)).Y];
            X2 = interp1(Y_jj,X_jj,Y_ii,'linear',inf);
            distError = X_ii-X2;
        end

        if abs(DNorm2(distError)) < 1e-5 && ii~=idxDK(jj)
            disp('=========================')
            disp(ii)
            disp(jj)
            disp(' ')
            idxRemove(nn)=ii;
            nn=nn+1;
            figure(1)
            hold on
            plot(X_jj,Y_jj)
            plot(X_ii,Y_ii)
            hold off
            pause()
            clf
            break
        end

    end
end
toc
end

function StateLattice=removeTurnOnTheSpot(StateLattice)
vertices=[[StateLattice.x0].',[StateLattice.y0].',[StateLattice.x1].',[StateLattice.y1].'];
idx=ismember(vertices,[0 0 0 0],'rows');
StateLattice(idx)=[];
end