function plotPath(Path,varargin)
%PLOTPATH Summary of this function goes here
%   Detailed explanation goes here
%   Notes :
%   * quickly plots in one shot thanks to nan (could also be inf)
co=get(groot,'DefaultAxesColorOrder');

if nargin == 1
    plotColor = co(1,:);
    lineWidthPlot=1;
    plotStyle='-';
elseif nargin == 2
    plotColor=(varargin{1});
    lineWidthPlot=1;
    plotStyle='-';
elseif nargin == 3
    plotColor=(varargin{1});
    lineWidthPlot=(varargin{2});
    plotStyle='-';
elseif nargin == 4
    plotColor=(varargin{1});
    lineWidthPlot=(varargin{2});
    plotStyle=(varargin{3});
end

PathFree=Path([Path.free]);

for ii=1:length(Path)
    if ~Path(ii).free
        idxPlot=Path(ii).idxBlocked;
        Path(ii).X=Path(ii).X(1:idxPlot-1);
        Path(ii).Y=Path(ii).Y(1:idxPlot-1);
        Path(ii).TH=Path(ii).TH(1:idxPlot-1);
    end
end


% [Path_Circ,Path_Cloth] = sortPathPlot(Path);
% X_Circ=zeros(1);
% Y_Circ=zeros(1);
% kk=0;
% for ii=1:length(Path_Circ)
%     kk=kk(end)+(1:(length(Path_Circ(ii).X)+1));
%     X_Circ(kk)=[Path_Circ(ii).X;nan];
%     Y_Circ(kk)=[Path_Circ(ii).Y;nan]; 
% end
% X_Cloth=zeros(1);
% Y_Cloth=zeros(1);
% kk=0;
% for ii=1:length(Path_Cloth)
%     kk=kk(end)+(1:(length(Path_Cloth(ii).X)+1));
%     X_Cloth(kk)=[Path_Cloth(ii).X;nan];
%     Y_Cloth(kk)=[Path_Cloth(ii).Y;nan]; 
% end

% X_Cloth=zeros(1);
% Y_Cloth=zeros(1);
% kk=0;
% for ii=1:length(Path_Cloth)
%     kk=kk(end)+(1:(length(Path_Cloth(ii).X)+1));
%     X_Cloth(kk)=[Path_Cloth(ii).X;nan];
%     Y_Cloth(kk)=[Path_Cloth(ii).Y;nan]; 
% end

X_curve=zeros(1);
Y_curve=zeros(1);
kk=0;
for ii=1:length(Path)
    kk=kk(end)+(1:(length(Path(ii).X)+1));
    X_curve(kk)=[Path(ii).X;nan];
    Y_curve(kk)=[Path(ii).Y;nan]; 
end

XY=unique([[PathFree.x0].',[PathFree.y0].';[PathFree.x1].',[PathFree.y1].'],'rows');
    
plot(X_curve,Y_curve,plotStyle,'Color',plotColor,'Linewidth',lineWidthPlot);  
% plot(X_Circ,Y_Circ,'Color',co(2,:),'Linewidth',2);

% if ~isempty(XY) && size(XY,1)>5; plot(XY(:,1),XY(:,2),'k*','MarkerSize',15); end


%% I have to have MotPrem settings to plot this !
%     for ii=1:length(Path)
%         x1_ii=Path(ii).x1;
%         y1_ii=Path(ii).y1;
%         th1_ii=Path(ii).th1;
%         if rem(x1_ii,dx_SL) == 0 && rem(y1_ii,dx_SL) == 0 
%             scatter(x1_ii,y1_ii,300,'g*')
%         end
% 
%     end
end

