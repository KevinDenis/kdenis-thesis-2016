
co=get(groot,'DefaultAxesColorOrder');
% Example 3: dark blue - dark red colorbar, 20 colors, 5% darkest values and 20% lightest values cut
figure(1); pcolor([0:7;0:7]);
% colormap([flipud(cmap('Blue',4,0,30));cmap('red',4,0,30)]); colorbar('horiz');



Color7=rgb('DeepSkyBlue');
Color6=rgb('RoyalBlue');
Color5=rgb('MidnightBlue');
Color4=rgb('Black');
Color3=rgb('DarkRed');
Color2=rgb('FireBrick');
Color1=co(2,:);

colormap([Color1;Color2;Color3;Color4;Color5;Color6;Color7])