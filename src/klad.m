initWorkspace
load('MP_cloth.mat')
OT=ObstacleTable;
n=length(OT);

nn=1;
toDelete=zeros(1);
for ii = 1:n
    
    idx_ii=[OT(ii).Idx];
    
    if all(idx_ii==1)
        toDelete(nn)= ii;
        nn=nn+1;
    end
    
    
end

OT(toDelete)=[];
n=length(OT);
for ii = 1:n
    ID_ii=[OT(ii).ID];
    
    for jj=1:n
        ID_jj=[OT(jj).ID];
        if jj~=ii
            if all(ismember(ID_jj,ID_ii))
                disp('found a subset')
            end
            
            
        end
        
    end
    
    
    
    
    
    
    
    
    



end