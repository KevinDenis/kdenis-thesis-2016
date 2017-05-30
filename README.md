# Path planning algorithm for semi-autonomous mobile robots with fast and accurate collision checking
Master thesis of Kevin Denis, under supervision of Professor Demeester and Professor Bruyninckx, mentored by Johan Philips. 

## Goals
The goal of this master thesis is to design a local path planner which generates collision-free paths in a fast and accurate way.
This proposed method is a contribution to the Local Path Template developed by Prof. Demeester.
The goal is to overcome one of the shortcomings of the current version, the difficulty to plan feasible paths in narrow opening, 
due to the lack of flexibility of the employed curve geometry. 
In its current design, only circular curves are used as basis for the trajectories.
This thesis will expand the set of feasible motions by using a more complex curve geometry, 
thereby offering a higher flexibility.
By using a fixed set of trajectories, the occupancy grid of each path can be stored in a look-up table.
The online phase will consist in matching obstacles in the environment with this look-up table, and adjust the length of each path to be collision-free.
The case study for this Local Path Planner is Navigation Assistance of electrical powered wheelchair.
To provide an improved assistance, the intention of the driver has to be evaluated.
The collision-free paths will model the user intention.

## Git organization

### src (Source code)
Here, all the source code used for this thesis is presented.

### adm (Administration)
Here, all the administrative mather for this thesis is stored (e.g. logs, meeting rapport, etc.).

### ppr (Paper)
Here, the source code for the paper and pdf's will be available.

### prs	(Presentation)
Here, the source code for the presentation (mid-term and final) and pdf's will be available.