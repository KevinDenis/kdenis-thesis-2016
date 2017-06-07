function plotPath(Path,varargin)
%plotPath(Path,varargin)
%   Notes :
%   * quickly plots in one shot thanks to nan (could also be inf). This
%   improves considerably the time needed to plot large sets of paths

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

% Adjust path length if blocked somewhere
for ii=find(~[Path.free])
    idxPlot=Path(ii).idxBlocked;
    Path(ii).X=Path(ii).X(1:idxPlot-1);
    Path(ii).Y=Path(ii).Y(1:idxPlot-1);
    Path(ii).TH=Path(ii).TH(1:idxPlot-1);
end

X_curve=zeros(1);
Y_curve=zeros(1);
kk=0;
for ii=1:length(Path)
    kk=kk(end)+(1:(length(Path(ii).X)+1));
    X_curve(kk)=[Path(ii).X;nan];
    Y_curve(kk)=[Path(ii).Y;nan]; 
end

    
plot(X_curve,Y_curve,plotStyle,'Color',plotColor,'Linewidth',lineWidthPlot);  




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

