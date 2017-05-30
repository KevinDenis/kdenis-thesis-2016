# Source Code
this file containts the comments of the main files which gives an overview of the developed local path planner.

## main.m
This file is builds the local path template using a Local State Lattice and an obstacle based look-up table for a fast obstacle checking.
Several plots are shown to the user to evaluate and to inspect the obtained trajectories.

### User settings
* use pre-coomputed data y/n (bool value usePrecomputedData) 
* save data in data directory y/n (bool value on line 35) 
* select curve geometry {[1],2,3} to select {[Clothoid], Circular, Bézier}. (int value saveCalculatedData) 
* select map {[1],2,3,4} to select (int value selectMap) {[RobotLab_Elevator], RobotLab, RobotLab_ZoomEntrance, Elevator}') 
* see Local Path Planning y/n (bool value seeLPP) 
 
### Overview (more information is given in each function) 
* initWorkspace : adds needed directories to path for help functions 
* getLocalStateLatticeSettings : loads additional settings for the LSL
* BuildMultiSizeGrid : creates a multi-size grid based on LSLsettings 
* BuildLSLWithClothoids : Build Local State Lattice based on clothoids
* BuildLSLWithCircles : Build Local State Lattice based on circles 
* BuildLSLWithBezierCurve : Build LSL based on Bézier Curves 
* BuildOccGridFromLSL : calculates the occupancy of the wheelchair going over each set of paths of the Local State Lattice. This can take up to 5min. Path based Occupancy grid will be converted to obstacle based occupancy grid (quite efficiently, ~10sec) 
* BuildLSLColFree : given a map of the environment and a user-selected robot pose, paths lengths are adjusted to be collision-free. This is also plotted. If the user desires to check whether all plotted paths are collision free, please uncomment plotRobotPath(LSL_W) at the bottom of the function, in the plotting section. This will plot the robot shape over all collision-free paths. 
* LocalPathPlanning : OPTIONAL, set seeLPP to true to see the robot shape move over a selected path by the user. This mimics the role of the plan recognition algorithm. This can also be used to ensure that the path taken is obstacle free, by visual inspection. 
 
### TODO
* Calculate Bézier Curve LSL (only Motion Primitive at the moment) 
* Make backups who can't be overwritten by user accidently


## BuildMultiSizeGrid.m
creates a multi-size grid based on LSLsettings
### Overview 
* Builds a multi size grid based on the LSLset structure 
* 3 grid sizes are used, fine close to the origin and coarse far away 
* A end pose in the coarse grid represents thus a larger group of possible end poses compared to the fine grid. This emphasizes the fact that it is not useful to keep a large set of (relatively) similar paths with a end pose that is far from the origin. 
* each grid size within the Region of Interest (see next step) will be connected with the selected curve geometry. If this curve is feasible, it will be added to the set of paths of the Local State Lattice.
 grid 1 : 0.10x0.10 --> 2.0m x 2.0m (Fine grid)
 grid 2 : 0.25x0.25 --> 4.0m x 4.0m (Medium grid)
 grid 3 : 0.50x0.50 --> 8.0m x 6.0m (Coarse grid)
 
## BuildLSLWithClothoids.m
Build Local State Lattice based on clothoids

### Overview
* Discrete end poses defined by the multi size grid are connected with a clothoid. If this clothoid is compliant with the constraints, it is added to the Local State Lattice Structure containing all feasible paths starting from the robot actual position0. 
* First, every CEP defined by the ROI at the origin [0 0 0°] 
* Then this step is repeated at every feasible Expansion Position (EP) defined by dxEP. At these discrete poses, the first step is repeated, connecting feasible paths with this EP, creating a larger variety of paths. 
* Paths within the local state lattice set are therefore defined by 1 or 2 clothoids.

### IMPORTANT
Code to generate clothoids is not written by Kevin DENIS, but originates from : https://github.com/ebertolazzi/G1fitting mex files for fast computation are pre-compiled for Linux and Windows


## BuildOccGridFromLSL.m (Path and Obstacle Based) 
Calculates the occupancy of the wheelchair going over each set of paths of the Local State Lattice. This can take up to 5min. Path based Occupancy grid will be converted to obstacle based occupancy grid (quite efficiently, ~10sec) 

### Overview 
* First, the traditional "Path Based" Occupancy grid is calculated, since this is the most logical way for generating an occupancy grid of each path, by letting the shape of the robot move over the paths calculated in the previous step.This part updates the State Latice Structure. 
* Secondly, the Obstacke Based Occupancy grid is generated using the previously calculated data. A clever trick is used in order to generate his obstacle based occupancy grid in an efficient way. This saves a lot of computing time, compared to a naive search based method. Clever trick exmplained below, in "Obstacle Based Approach". This part of the code will create a new structure, ObstacleTable. A Matrix form of this structure will also be kept, XY_ObsTable ObstacleTable contains all paths where the robot occupies a certain x-y position and at what idx ("time") this happens. 

## BuildLSLColFree.m 
Given a map of the environment and a user-selected robot pose, paths lengths are adjusted to be collision-free. This is also plotted. If the user desires to check whether all plotted paths are collision free, please uncomment plotRobotPath(LSL_W) at the bottom of the function, in the plotting section. This will plot the robot shape over all collision-free paths. 

### Overview 
* The user selects the start position of the robot followed by it's ordientation 
* Given the user-selected map, and the user-selected robot pose, the map is translated in the robot coordinate frame {R}. This because the Local Path Template / ObstacleTable / Local State Lattice is formulated according to this Local Reference Frame {R}, which asumes that the robot is at the orogin [0 0 0°]. 
* Clossiion free trajectories are calculated efficiently and accuratly by using the pre-cumputed obstacle table, which indicates for each dircrete grid which paths goes through it at which length. The online fase only has to mach occupied cells with the obstacle table, and add just each path length(only if this results in a smaller path 
* Once evevry path is update, the paths are translated in the World Coordinates {W} for a better readability for the user (LSL_W) 


## LocalPathPlanning.m 
OPTIONAL, set seeLPP to true to see the robot shape move over a selected path by the user. This mimics the role of the plan recognition algorithm. This can also be used to ensure that the path taken is obstacle free, by visual inspection.
### Overview 
 * This function will mimic the plan recognition algorithm. The user selects a path by clicking on it. The closest path to this input will be chosen. If this path is not directly connected to the robot pose, the path connecting the robot pose and the destination is searched. Since one path is composed at most by 2 subsequent curves, this process is only done once 
 * In a previous version of this code, Dijkstra’s algorithm was used to do this. This was a bit "overkill" since in this case "a" solution is also the "optimal" solution. Only one path leads to a certain intermediate destination (the expansion position), where the second curve originates. This has been commented out at the end of this function 
 


 Kevin DENIS, KU Leuven, 2016-17 
 Master Thesis: Path planning algorithm for semi-autonomous 
 mobile robots with fast and accurate collision checking