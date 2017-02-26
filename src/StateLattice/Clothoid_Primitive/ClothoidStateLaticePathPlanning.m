clear
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                           %
%       State Latice Matrix Structure       %
%  [ x0 y0 th0 x1 y1 th1 k dk L intKappa ]  %
%    1  2   3  4  5   6  7  8 9    10       %
%                                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% TODO
% * faster assigment for sparse matrix, by computing ii,jj beforhand
% * Not all start positions are end positions
% * Not only spiegeling but also roation
NPTS=100;
load('ClothoidStateLatice_Test.mat')
addpath('ClothoidG1fitting');


x_sift_vec=-4:1:4;
y_sift_vec=-4:1:4;
EdgesMatrix=ClothoidStateLaticeFull;
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
        StateLaticeMatrix_Augmented(k,1:end-4)=[x_s_tmp y_s_tmp th_s_tmp x_d_tmp y_d_tmp th_d_tmp];
        k=k+length(EdgesMatrix);
    end
end
length(StateLaticeMatrix_Augmented)
[StateLaticeMatrix_Augmented] = unique(StateLaticeMatrix_Augmented,'rows'); % normaly not needed ! x_sift and y_sift are unique
length(StateLaticeMatrix_Augmented)
%% Building vertices
vx=StateLaticeMatrix_Augmented(:,[1 4]);
vy=StateLaticeMatrix_Augmented(:,[2 5]);
vth=StateLaticeMatrix_Augmented(:,[3 6]);

%% construct graph
xyth_all = unique([vx(:) vy(:) vth(:)], 'rows');

xyth_s = [vx(:,1) vy(:,1) vth(:,1)];
ii_temp =  findVector(xyth_all,xyth_s);
xyth_d = [vx(:,2) vy(:,2) vth(:,2)];
jj_temp =  findVector(xyth_all,xyth_d);
ii=[ii_temp;jj_temp];
jj=[jj_temp;ii_temp];
dispCost=1*StateLaticeMatrix_Augmented(:,end)+3*StateLaticeMatrix_Augmented(:,end-1);
% dispCost=[dispCost;dispCost];
% dispCost
% G = sparse(length(xyth_all),length(xyth_all));
tic
G = sparse(ii_temp,jj_temp,dispCost,length(xyth_all),length(xyth_all));
toc
% figure()
% spy(G)


%%
tic
xyth_start=[-2 -2 pi/2];
xyth_end=[2 2 0];


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
    optimal_edges(ii)=findVectorRev(All_Edges,v_sd);
end
toc
%% plot optimal path

xy_grid = getAllComb(x_sift_vec,y_sift_vec);

if true
    figure(1)
    hold on
    scatter(xy_grid(:,1),xy_grid(:,2),[],'k*')
%     scatter(xyth_start(1),xyth_start(2),[],'g*')
%     scatter(xyth_end,xyth_end,[],'g*')
%     for ii=1:2:length(StateLaticeMatrix_Augmented)
%         x0=StateLaticeMatrix_Augmented(ii,1);
%         y0=StateLaticeMatrix_Augmented(ii,2);
%         th0=StateLaticeMatrix_Augmented(ii,3);
%         k=StateLaticeMatrix_Augmented(ii,7);
%         dk=StateLaticeMatrix_Augmented(ii,8);
%         Lsol=StateLaticeMatrix_Augmented(ii,9); 
%         [X,Y] = pointsOnClothoid(x0,y0,th0,k,dk,Lsol,NPTS) ; % initial point and direction clothoid parameters
%         plot(X,Y,'k')   
%     end
    for ii=1:length(optimal_edges)
        kk=optimal_edges(ii);
        x0=StateLaticeMatrix_Augmented(kk,1);
        y0=StateLaticeMatrix_Augmented(kk,2);
        th0=StateLaticeMatrix_Augmented(kk,3);
        x1=StateLaticeMatrix_Augmented(kk,4);
        y1=StateLaticeMatrix_Augmented(kk,5);
        th1=StateLaticeMatrix_Augmented(kk,6);
        k=StateLaticeMatrix_Augmented(kk,7);
        dk=StateLaticeMatrix_Augmented(kk,8);
        Lsol=StateLaticeMatrix_Augmented(kk,9); 
        [X,Y] = pointsOnClothoid(x0,y0,th0,k,dk,Lsol,NPTS) ; % initial point and direction clothoid parameters
        plot(X,Y,'g','LineWidth',3)
        scatter(x0,y0,[],'r*')
        scatter(x1,y1,[],'r*')
    end
    grid on
    hold off
    l1=legend('shifted motion primitive','optimal path','visited nodes','Location','NW');
    set(l1,'FontSize',12);
end
%}