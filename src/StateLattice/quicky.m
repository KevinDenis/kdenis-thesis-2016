clear all
close all
clc

% Fancy multi bar (use labels and update all bars at once)
m = 4;
n = 3;
p = 100;
progressbar('Overall State Lattice Progress','Current Manhatten Distance Progress') % Init 2 bars

for j = 1:n
    for k = 1:p
        pause(0.01) % Do something important
        % Update all bars
        frac3 = k/p;
        frac2 = ((j-1) + frac3) / n;
        progressbar(frac2, frac3)
    end
end
