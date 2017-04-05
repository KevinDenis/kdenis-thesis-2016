% Notes :
%   *
%   *

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%                        State Latice Structure                           %
%[ x0 y0 th0 x1 y1 th1 X Y TH k dk Ltot intK PathOccXY pathCost free ID ] %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


initWorkspace

showPlot=false;

% grid 1 : 0.1x0.1 --> 1.5m x 1.5m
% grid 2 : 0.25x0.25 --> 2m
% grid 3 : 0.5x0.5 --> 4m

LSLset.x0=0; 
LSLset.y0=0;
LSLset.th0=0;
LSLset.xmax=2.49;
LSLset.ymax=1.49;
LSLset.kmax=1;
LSLset.res = 5e-2;
LSLset.dth=pi/8;
LSLset.dxSL=0.5;
LSLset.ROI=[0 LSLset.ymax; LSLset.xmax LSLset.ymax; LSLset.xmax -LSLset.ymax;0 -LSLset.ymax;0 LSLset.ymax];

%
% ^     [3 1]       [3 3]
% |     [2 1] [2 2]
% y     [1 1] [1 2] [1 3]
%   x -->


% GRID 1
dx1=0.1;
x1_max=1;
X11=0:dx1:x1_max;
Y1=X11;
grid_XY1=getAllComb(X11,Y1);

% GRID 2
dx2=0.25;
x2_max=2;
X21=0:dx2:x1_max;
X22=x1_max+dx2:dx2:x2_max; 
X12=X22;
Y22=X22;
Y21=Y22;
Y12=X21;
grid_XY21=getAllComb(X21,Y21);
grid_XY22=getAllComb(X22,Y22);
grid_XY12=getAllComb(X12,Y12);
grid_XY2=[grid_XY21;grid_XY22;grid_XY12];

% GRID 3
dx3=0.5;
x3_max=6;
X31=0:dx3:x3_max;
X33=x2_max+dx3:dx3:x3_max; 
X13=X33;
Y33=X33;
Y31=Y33;
Y13=X31;
grid_XY31=getAllComb(X31,Y31);
grid_XY33=getAllComb(X33,Y33);
grid_XY13=getAllComb(X13,Y13);
grid_XY3=[grid_XY31;grid_XY33;grid_XY13];

% FULL GRID
grid_XY=[grid_XY1;grid_XY2;grid_XY3];
grid_XY=unique(grid_XY,'rows');
grid_XY=[grid_XY(:,1) grid_XY(:,2); grid_XY(:,1) -grid_XY(:,2)];
grid_XY=unique(grid_XY,'rows');
grid_XY=[grid_XY(:,1) grid_XY(:,2); -grid_XY(:,1) grid_XY(:,2)];
grid_XY=unique(grid_XY,'rows');

ROI0=LSLset.ROI;
idxIn0=inpolygon(grid_XY(:,1),grid_XY(:,2),ROI0(:,1),ROI0(:,2));

ROI1=RotTransXY(ROI0,1.1781,1.5,1);
idxIn1=inpolygon(grid_XY(:,1),grid_XY(:,2),ROI1(:,1),ROI1(:,2));

if showPlot
    co=get(groot,'DefaultAxesColorOrder');


    
    figure()
    title('Multi-size grid')
    hold on
    axis equal
    plotGrid(grid_XY,[0 0 0])
    l=legend('discrete grids');set(l,'FontSize',12);
%     pause()
%     saveCurrentFigure
    
    figure()
    title('Multi-size grid, with different robot poses')
    hold on
    axis equal
    plot(grid_XY(:,1),grid_XY(:,2),'o','Color',[0.5 .5 .5])
    plot(ROI0(:,1),ROI0(:,2),'go-','Linewidth',2)
    plot(grid_XY(idxIn0,1),grid_XY(idxIn0,2),'og')
    plot(ROI1(:,1),ROI1(:,2),'ro-','Linewidth',2)
    plot(grid_XY(idxIn1,1),grid_XY(idxIn1,2),'ro')
    set(l,'FontSize',12);
    plotSimpleRobot([0 0 0;1.5, 1, 1.1781])
    l=legend('discrete grids','ROI0','Grids In ROI0','ROI1','Grids in ROI1','robot pose');
%     pause()
%     saveCurrentFigure
end