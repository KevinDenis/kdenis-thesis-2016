clear
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%                       State Latice Matrix Structure                     %
%[ x_start y_start th_start x_end y_end th_end x_ctr y_ctr dist intKappa ]%
%     1       2       3       4     5     6    7:10  11:14  15     16     %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% TODO
% * faster assigment for sparse matrix, by computing ii,jj beforhand
% * Not all start positions are end positions
% * Not only spiegeling but also roation

% load('StateLatice_Medium.mat')
load('StateLatice_Small.mat')



EdgesMatrix_Q1=zeros(length(stateLatice),16);
for ii=1:length(stateLatice)
    EdgesMatrix_Q1(ii,3)=stateLatice(ii).th1;
    EdgesMatrix_Q1(ii,4)=stateLatice(ii).xend;
    EdgesMatrix_Q1(ii,5)=stateLatice(ii).yend;
    EdgesMatrix_Q1(ii,6)=stateLatice(ii).thend;
    EdgesMatrix_Q1(ii,7:10)=stateLatice(ii).x;
    EdgesMatrix_Q1(ii,11:14)=stateLatice(ii).y;
    EdgesMatrix_Q1(ii,15)=stateLatice(ii).s;
    EdgesMatrix_Q1(ii,16)=stateLatice(ii).intKappaT;
end

EdgesMatrix_Q2=zeros(size(EdgesMatrix_Q1));
for ii=1:length(stateLatice)
    EdgesMatrix_Q2(ii,3)=pi-stateLatice(ii).th1;
    EdgesMatrix_Q2(ii,4)=-stateLatice(ii).xend;
    EdgesMatrix_Q2(ii,5)=stateLatice(ii).yend;
    EdgesMatrix_Q2(ii,6)=pi-stateLatice(ii).thend;
    EdgesMatrix_Q2(ii,7:10)=-stateLatice(ii).x;
    EdgesMatrix_Q2(ii,11:14)=stateLatice(ii).y;
    EdgesMatrix_Q2(ii,15)=stateLatice(ii).s;
    EdgesMatrix_Q2(ii,16)=stateLatice(ii).intKappaT;
end

EdgesMatrix_Q3=zeros(size(EdgesMatrix_Q1));
for ii=1:length(stateLatice)
    EdgesMatrix_Q3(ii,3)=-pi+stateLatice(ii).th1;
    EdgesMatrix_Q3(ii,4)=-stateLatice(ii).xend;
    EdgesMatrix_Q3(ii,5)=-stateLatice(ii).yend;
    EdgesMatrix_Q3(ii,6)=-pi+stateLatice(ii).thend;
    EdgesMatrix_Q3(ii,7:10)=-stateLatice(ii).x;
    EdgesMatrix_Q3(ii,11:14)=-stateLatice(ii).y;
    EdgesMatrix_Q3(ii,15)=stateLatice(ii).s;
    EdgesMatrix_Q3(ii,16)=stateLatice(ii).intKappaT;
end

EdgesMatrix_Q4=zeros(size(EdgesMatrix_Q1));
for ii=1:length(stateLatice)
    EdgesMatrix_Q4(ii,3)=-stateLatice(ii).th1;
    EdgesMatrix_Q4(ii,4)=stateLatice(ii).xend;
    EdgesMatrix_Q4(ii,5)=-stateLatice(ii).yend;
    EdgesMatrix_Q4(ii,6)=-stateLatice(ii).thend;
    EdgesMatrix_Q4(ii,7:10)=stateLatice(ii).x;
    EdgesMatrix_Q4(ii,11:14)=-stateLatice(ii).y;
    EdgesMatrix_Q4(ii,15)=stateLatice(ii).s;
    EdgesMatrix_Q4(ii,16)=stateLatice(ii).intKappaT;
end


EdgesMatrix=[EdgesMatrix_Q1;EdgesMatrix_Q2;EdgesMatrix_Q3;EdgesMatrix_Q4];
EdgesMatrix(:,[3 6])=wrapToPi(EdgesMatrix(:,[3 6]));

[~,idx,~] = unique(EdgesMatrix(:,1:6),'rows'); EdgesMatrix=EdgesMatrix(idx,:);

x_sift_vec=-2:2:2;
y_sift_vec=-2:2:2;

StateLaticeMatrix_Augmented=repmat(EdgesMatrix,length(x_sift_vec)*length(y_sift_vec),1);
k=1:length(EdgesMatrix);
for x_sift=x_sift_vec
    for y_sift=y_sift_vec
        x_s_tmp=StateLaticeMatrix_Augmented(k,1)+x_sift;
        y_s_tmp=StateLaticeMatrix_Augmented(k,2)+y_sift;
        th_s_tmp=StateLaticeMatrix_Augmented(k,3);
        x_d_tmp=StateLaticeMatrix_Augmented(k,4)+x_sift;
        y_d_tmp=StateLaticeMatrix_Augmented(k,5)+y_sift;
        th_d_tmp=StateLaticeMatrix_Augmented(k,6);      
        x_Bezier_tmp=StateLaticeMatrix_Augmented(k,7:10)+x_sift;
        y_Bezier_tmp=StateLaticeMatrix_Augmented(k,11:14)+y_sift;
        StateLaticeMatrix_Augmented(k,1:end-2)=[x_s_tmp y_s_tmp th_s_tmp x_d_tmp y_d_tmp th_d_tmp x_Bezier_tmp y_Bezier_tmp];
        k=k+length(EdgesMatrix);
    end
end
[StateLaticeMatrix_Augmented] = unique(StateLaticeMatrix_Augmented,'rows'); % normaly not needed ! x_sift and y_sift are unique

%% Building vertices
vx=StateLaticeMatrix_Augmented(:,[1 4]);
vy=StateLaticeMatrix_Augmented(:,[2 5]);
vth=StateLaticeMatrix_Augmented(:,[3 6]);

%% construct graph
xyth_all = unique([vx(:) vy(:) vth(:)], 'rows');
% dv = [vx_new(1,:); vy_new(1,:)] - [vx_new(2,:); vy_new(2,:)];
% edge_dist = sqrt(sum(dv.^2));

G = sparse(length(xyth_all),length(xyth_all));
for kk = 1:length(StateLaticeMatrix_Augmented)
    xyth_s = [vx(kk,1) vy(kk,1) vth(kk,1)];
    ii =  findVector(xyth_all,xyth_s);
    xyth_d = [vx(kk,2) vy(kk,2) vth(kk,2)];
    jj =  findVector(xyth_all,xyth_d);
    G(ii,jj) = StateLaticeMatrix_Augmented(kk,end);
    G(jj,ii) = StateLaticeMatrix_Augmented(kk,end);
end

%%
xyth_start=[-2 -2 0];
xyth_end=[4 4 -pi/4];


%% starting index and destination index
st_idx = findVector(xyth_all,xyth_start);
dest_idx = findVector(xyth_all,xyth_end);
All_Edges = StateLaticeMatrix_Augmented(:,1:6);
%% dijestra's algoritm to find shortes path
[dist,path,pred] = graphshortestpath(G,st_idx,dest_idx);
xyth_opt_path = xyth_all(path,:);

optimal_edges = zeros(size(xyth_opt_path,1)-1,1);
for ii=1:size(xyth_opt_path,1)-1
    v_sd=[xyth_opt_path(ii,:) xyth_opt_path(ii+1,:)];   
    optimal_edges(ii)=findVector(All_Edges,v_sd);
    disp(ii)
end

%% plot optimal path

if true
    figure()
    hold on
    for ii=1:length(EdgesMatrix_Q1)
        x_temp=EdgesMatrix_Q1(ii,7:10);
        y_temp=EdgesMatrix_Q1(ii,11:14);
        B=BezierCurve(x_temp,y_temp);
        plot(B(:,1),B(:,2),'k')   
    end
    hold off

    
    
    figure()
    hold on
    for ii=1:length(EdgesMatrix)
        x_temp=EdgesMatrix(ii,7:10);
        y_temp=EdgesMatrix(ii,11:14);
        B=BezierCurve(x_temp,y_temp);
        plot(B(:,1),B(:,2),'k')   
    end
    hold off


    figure()
    hold on
    for ii=1:length(StateLaticeMatrix_Augmented)
        x_temp=StateLaticeMatrix_Augmented(ii,7:10);
        y_temp=StateLaticeMatrix_Augmented(ii,11:14);
        B=BezierCurve(x_temp,y_temp);
        plot(B(:,1),B(:,2),'k')   
    end
    for ii=1:length(optimal_edges)
        kk=optimal_edges(ii);
        x_temp=StateLaticeMatrix_Augmented(kk,7:10);
        y_temp=StateLaticeMatrix_Augmented(kk,11:14);
        B=BezierCurve(x_temp,y_temp);
        plot(B(:,1),B(:,2),'g','LineWidth',3)
    end
    for x_sift=x_sift_vec
        for y_sift=y_sift_vec
            scatter(x_sift,y_sift,[],'r*')
        end
    end
    hold off
end