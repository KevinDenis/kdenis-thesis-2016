# Source Code 
this file containts the comments of the main files which gives an overview of the developed local path planner.

## Main
This file is builds the local path template using a Local State Lattice and an obstacle based look-up table for a fast obstacle checking.

### User settings :                                                         
* use pre-coomputed data y/n (bool value usePrecomputedData)          
* save data in data directory y/n (bool value on line 35)             
* select curve geometry {[1],2,3} to select {[Clothoid], Circular, Bézier}. (int value saveCalculatedData)      
* select map {[1],2,3,4} to select (int value selectMap) {[RobotLab_Elevator], RobotLab, RobotLab_ZoomEntrance, Elevator}')  
* see Local Path Planning y/n (bool value seeLPP)                     
                                                                         
### Overview (more information is given in each function) :                                                                                         
* initWorkspace : adds needed directories to path for help functions  
* getLocalStateLatticeSettings : loads additional settings for the LSL
* BuildLSLWithClothoids : Build Local State Lattice based on clothoids
* BuildLSLWithCircles : Build Local State Lattice based on circles    
* BuildLSLWithBezierCurve : Build LSL based on Bézier Curves          
* BuildOccGridFromLSL : calculates the occupancy of the wheelchair going over each set of paths of the Local State Lattice. This can take up to 5min. Path based Occupancy grid will be converted to obstacle based occupancy grid (quite efficiently, ~10sec)               
* BuildLSLColFree : given a map of the environment and a user-selected robot pose, paths lengths are adjusted to be collision-free. This is also plotted. If the user desires to check whether all plotted paths are collision free, please uncomment plotRobotPath(LSL_W) at the bottom of the function, in the plotting section. This will plot the robot shape over all collision-free paths.                                                                                            
* LocalPathPlanning : OPTIONAL, set seeLPP to true to see the robot shape move over a selected path by the user. This mimics the role of the plan recognition algorithm. This can also be used to ensure that the path taken is obstacle free, by visual inspection.                                                                                      
 
### TODO :                                                                  
   * Calculate Bézier Curve LSL (only Motion Primitive at the moment)    
   * Make backups who can't be overwritten by user accidently            
                                                                         
 Kevin DENIS, KU Leuven, 2016-17                                         
 Master Thesis: Path planning algorithm for semi-autonomous              
                mobile robots with fast and accurate collision checking  
