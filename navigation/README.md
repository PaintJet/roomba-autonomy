# navigation
The navigation package implements the `move_base` package. The move_base package handles multiple components of the autonomous software

* Converting the occupancy grid to a costmap. The costmap defines the cost for the robot occupying each grid cell
* Adding dynamic obstacles to costmap layers using sensor data
* Using the global planner to define waypoints to reach the defined goal.
* Using the local planner to send motor commands to the drive motors.

While we have implemented this package, all of the parameters are the default tutorial parameters. These parameters definitely need to be tuned.

## Known Issues
* Costmap layers beyond static layer have not been implemented yet
* Robot does not appear to avoid obstacles in the static layer
* Global Planner is not well configured - needs to be customized for our application of it
* Tolerances for the Local Planner are too high and continues rotating around goal, never reaching it
* Not enough testing has been done with this package