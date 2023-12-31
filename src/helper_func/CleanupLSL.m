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

%% A path must lead to a different lattice
vertices = getStartEndVerticesPath(Path);
toDelete = all(vertices(:,1:3) == vertices(:,4:6),2);
Path(toDelete)=[];

%% A path must have a unique start and end destination
vertices = getStartEndVerticesPath(Path);
[vertices,idxUnique,~]=unique(vertices,'rows','stable');
Path=Path(idxUnique);

%% A edge which has a start vertex and end vertex which is reached by a same previous start vertex should be deleted
toDelete=zeros(1);
nn=1;
for ii=2:size(vertices,1)
    % gather all vertices before current vertex
    vertices_prev=vertices(1:ii-1,:);

    % gather current vertex
    vertex_ii=vertices(ii,:);
    
    idxEnd=findrow_mex(vertices_prev(:,4:6),vertex_ii(4:6));
    if ~isnan(idxEnd) % this destination has allready been reached !
        startVertexPrev=vertices_prev(idxEnd,1:3); % start (previous) vertex connecting the current desination
        idxStartOldToStartNew=findrow_mex(vertices_prev,[startVertexPrev vertex_ii(1:3)]);
        if ~isnan(idxStartOldToStartNew) % there is a connection between start and current start
            toDelete(nn)=ii; % this current path doesn't contribute to the set and therefore be deleted
            nn=nn+1;
        end
    end
end

if (toDelete(1) ~= 0); Path(toDelete)=[]; end

% assign unique path ID
Path_Cleaned=Path;
for ii=1:length(Path_Cleaned)
    Path_Cleaned(ii).ID=ii;
end

end

%% NOT IMPLEMENTED JET. A path contained by any other path should be deleted 
function keepLongestPaths()
tic
LSL_fwd = getForwardMotionFromStateLattice(LSL);
LSL_fwd=removeToSFromLSL(LSL_fwd);
dk_all=[LSL_fwd.dk];
idxRemove=zeros(1);
nn=1;
for ii=1:length(LSL_fwd)
    dk_ii=LSL_fwd(ii).dk;
    X_ii=LSL_fwd(ii).X;
    Y_ii=LSL_fwd(ii).Y;
    idxDK=find(ismembertol(dk_all,dk_ii,1e-3)==1);
    for jj=1:length(idxDK)
        X_jj=[LSL_fwd(idxDK(jj)).X];
        if ~IsNear(X_jj(1),X_jj(end),1e-3)
            Y_jj=[LSL_fwd(idxDK(jj)).Y];
            Y2 = qinterp1(X_jj,Y_jj,X_ii);
            distError = Y_ii-Y2;
        else
            Y_jj=[LSL_fwd(idxDK(jj)).Y];
            X2 = qinterp1(Y_jj,X_jj,Y_ii);
            distError = X_ii-X2;
        end
        if abs(DNorm2(distError)) < 1e-3 && ii~=idxDK(jj)
            idxRemove(nn)=ii;
            nn=nn+1;
            break
        end

    end
end
toc
end