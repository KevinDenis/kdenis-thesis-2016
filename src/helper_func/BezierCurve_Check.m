initWorkspace
load('LSL_bezier.mat')
length(LSL)

n=length(LSL);
for ii=1:n
    if LSL(ii).dk ~=0 % assignet if pure line
        K_ii=LSL(ii).K;
        dk_ii=DGradient(K_ii);
        LSL(ii).dk = mean(dk_ii);
    end
end


vertices=getStartEndVerticesPath(LSL);
idxOrigin=ismember(vertices(:,2:3), [0 0],'rows');

vertexOrigin=vertices(idxOrigin,:);

idxOriginQ1=vertexOrigin(:,5)>=0;

vertexOrigin_Q1=vertexOrigin(idxOriginQ1,:);

idxExcists=ismember(vertices(:,1:3),[0 0 0; vertexOrigin_Q1(:,4:6)],'rows');
idxOriginQ1=[idxOriginQ1;ones(length(idxExcists)-length(idxOriginQ1),1)];
selectSL=all([idxExcists,idxOriginQ1],2);
% find all the Q1 paths

LSL_Q1=LSL(selectSL);
LSL_Q4=LSL(selectSL);
n=length(LSL_Q1);

for ii=1:n
    LSL_Q4(ii).th0    = -wrap2Pi(LSL_Q1(ii).th0);
    LSL_Q4(ii).y1     =-LSL_Q1(ii).y1;
    LSL_Q4(ii).th1    = -wrap2Pi(LSL_Q1(ii).th1);
    LSL_Q4(ii).k      =-LSL_Q1(ii).k;
    LSL_Q4(ii).dk     =-LSL_Q1(ii).dk;
    LSL_Q4(ii).Y      =-LSL_Q1(ii).Y;
    LSL_Q4(ii).TH     =-wrap2Pi(LSL_Q1(ii).TH);
end

LSL=[LSL_Q1;LSL_Q4];
length(LSL)
LSL = CleanupLSL(LSL);
length(LSL)

n=length(LSL);
for ii=1:n
    S_ii=LSL(ii).S;
    jj_keep=[];
    next_ds=0;
    for jj=2:length(S_ii)-1
        if next_ds <= S_ii(jj)
            jj_keep=[jj_keep;jj-1];
            next_ds=next_ds+0.1;
        end
    end
    jj_keep=unique([jj_keep;length(S_ii)]);
    LSL(ii).X=LSL(ii).X(jj_keep);
    LSL(ii).Y=LSL(ii).Y(jj_keep);
    LSL(ii).TH=LSL(ii).TH(jj_keep);
    LSL(ii).S=LSL(ii).S(jj_keep);
    LSL(ii).K=LSL(ii).K(jj_keep);
end

LSL = FreeAllPaths(LSL);

figure()
title('')
hold on
grid on
xlabel('x [m]')
ylabel('y [m]')
axis equal
plotPath(LSL)
length(LSL)