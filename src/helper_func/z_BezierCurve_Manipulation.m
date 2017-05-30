initWorkspace
load('SL_bezier.mat')
length(StateLattice)

n=length(StateLattice);
for ii=1:n
    if StateLattice(ii).dk ~=0 % assignet if pure line
        K_ii=StateLattice(ii).K;
        dk_ii=DGradient(K_ii);
        StateLattice(ii).dk = mean(dk_ii);
    end
end


voxel=[[StateLattice.x0].',[StateLattice.y0].',abs([StateLattice.th0].'), ...
       [StateLattice.x1].',[StateLattice.y1].',abs([StateLattice.th1].')];

idxOrigin=ismember(voxel(:,2:3), [0 0],'rows');

voxelOrigin=voxel(idxOrigin,:);

idxOriginQ1=voxelOrigin(:,5)>=0;

voxelOrigin_Q1=voxelOrigin(idxOriginQ1,:);
% voxelOrigin_Y0=voxelOrigin(voxelOrigin(:,5)==0,:);


idxExcists=ismember(voxel(:,1:3),[0 0 0; voxelOrigin_Q1(:,4:6)],'rows');
idxOriginQ1=[idxOriginQ1;ones(length(idxExcists)-length(idxOriginQ1),1)];
selectSL=all([idxExcists,idxOriginQ1],2);
% find all the Q1 paths

StateLatticeQ1=StateLattice(selectSL);
StateLatticeQ4=StateLattice(selectSL);
n=length(StateLatticeQ1);

for ii=1:n
    StateLatticeQ4(ii).th0    = -wrap2Pi(StateLatticeQ1(ii).th0);
    StateLatticeQ4(ii).y1     =-StateLatticeQ1(ii).y1;
    StateLatticeQ4(ii).th1    = -wrap2Pi(StateLatticeQ1(ii).th1);
    StateLatticeQ4(ii).k      =-StateLatticeQ1(ii).k;
    StateLatticeQ4(ii).dk     =-StateLatticeQ1(ii).dk;
    StateLatticeQ4(ii).Y      =-StateLatticeQ1(ii).Y;
    StateLatticeQ4(ii).TH     =-wrap2Pi(StateLatticeQ1(ii).TH);
end

StateLattice=[StateLatticeQ1;StateLatticeQ4];
length(StateLattice)
StateLattice = CleanupStateLattice(StateLattice);
length(StateLattice)

n=length(StateLattice);
for ii=1:n
    S_ii=StateLattice(ii).S;
    jj_keep=[];
    next_ds=0;
    for jj=2:length(S_ii)-1
        if next_ds <= S_ii(jj)
            jj_keep=[jj_keep;jj-1];
            next_ds=next_ds+0.1;
        end
    end
    jj_keep=unique([jj_keep;length(S_ii)]);
    StateLattice(ii).X=StateLattice(ii).X(jj_keep);
    StateLattice(ii).Y=StateLattice(ii).Y(jj_keep);
    StateLattice(ii).TH=StateLattice(ii).TH(jj_keep);
    StateLattice(ii).S=StateLattice(ii).S(jj_keep);
    StateLattice(ii).K=StateLattice(ii).K(jj_keep);
end

StateLattice = FreeAllPaths(StateLattice);

figure()
title('')
hold on
grid on
xlabel('x [m]')
ylabel('y [m]')
axis equal
plotPath(StateLattice)
length(StateLattice)