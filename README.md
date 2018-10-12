# Robot Localization

Siena Okuno, Matthew Beaudouin, Nina Tchirkova

10/12/18
Computational Robotics

Olin College of Engineering

## Project Goal
The objective of this project was to create a working robot localization algorithm.

## Implementation Overview
Using ros-based processes, we implemented a particle filter node that contained 5 main object-oriented scripts. Within each of these were the different classes:TFHelper, Particle, ParticleCloud, OccupancyField, ParticleFilter, and RobotLocalizer. 

**These classes will be explained in detail later.**

**---------setup before localization: bag map, set up rviz**

The particle filter is actually run in robot_localizer.py through the run function. It starts off with the initialization of 500 particles with a direction, 2D location, and weight of 1 using the Particles class. These particles are the initial estimates for the robot's location given a known map, but are random at initialization.

When a new lidar scan is published, the function robot_localizer.get_x_directions(8) reduces the lidar scan to the values of 8  angles' ranges. These different angles are spaced equally between 0 and 2pi to have a reasonable amount of data for the robot's location without unneccessary run-time lag later.

With the data from the lidar scan, we can now manipulate the particles to test their probability of a location match. During the compare_points function, the various particles are assigned weights based on how accurate their estimated location is compared to the lidar scan values. For each particle, 8 points are generated at the previously mentioned angles that map to the lidar scan range using the ParticleCloud class and generate_points(). Each point, which is now just an x and y coordinate, is then passed into the get_closest_obstacle_distance function from occupancy_field.py. This returns the closest object to the point based on the map. The weight of the particle is one over the sum of all the points' error distances. This means that as the error distances approach zero, the weight will increase and the particle will be deemed more probable.

After the weights of the particles have been determined, the highest weighted particle is published to the "topparticle" topic. This is the current best estimate for the robot, but overtime it will become more accurate.


+ Resample new particles based on weight
+ Wait for odom to change
+ Transform resampled points with randomness
+ Repeat all but initial step


#### TFHelper
TFHelper was a class already given to us. We used it to help with the transforms required between the different map frames.
**---------show which transforms were used and what the final map was (like the diagram Paul drew)**
#### Particle and ParticleCloud
When initialized, the Particle class creates particles with a location, direction, and weight. Each can then be translated 
#### OccupancyField

#### ParticleFilter

#### RobotLocalizer


## Challenges
+ errors / compiling everything at the end

## Future Improvements
+ better communication
+ incremental testing
+ improving computation run-time vs accuracy

## Learnings
+ how a particle filter works
