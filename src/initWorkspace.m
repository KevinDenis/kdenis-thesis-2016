clearvars -except grid_XY XY_ObsTable LSLset LSL LSL_W ROI0 ROI1 idxIn0 idxIn1 robotPose LabGrid ObstacleTable XY_occ_lab_R path
close all
clc
addpath('helper_external');
addpath('helper_external/ClothoidG1fitting');
addpath('helper_func/BezierCurve');
addpath('helper_func');
addpath('helper_plot');
addpath('data_img');
addpath('data_mat');
% addpath('scripts_plot');