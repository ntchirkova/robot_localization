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

+ waits for odom movement

The particle filter is actually run in robot_localizer.py through the run function. It starts off with the initialization of 500 particles with a direction, 2D location, and weight of 1 using the Particles class. These particles are the initial estimates for the robot's location given a known map, but are random at initialization.

When a new lidar scan is published, the function robot_localizer.get_x_directions(8) reduces the lidar scan to the values of 8  angles' ranges. These different angles are spaced equally between 0 and 2pi to have a reasonable amount of data for the robot's location without unneccessary run-time lag later.

+ update_all_particles
+ draw_markerArray

With the data from the lidar scan, we can now manipulate the particles to test their probability of a location match. During the compare_points function, the various particles are assigned weights based on how accurate their estimated location is compared to the lidar scan values. For each particle, 8 points are generated at the previously mentioned angles that map to the lidar scan range using the ParticleCloud class and generate_points(). Each point, which is now just an x and y coordinate, is then passed into the get_closest_obstacle_distance function from occupancy_field.py. This returns the closest object to the point based on the map. The weight of the particle is one over the sum of all the points' error distances. This means that as the error distances approach zero, the weight will increase and the particle will be deemed more probable.

After the weights of the particles have been determined, the highest weighted particle is published to the "topparticle" topic. This is the current best estimate for the robot, but overtime it will become more accurate.

Then fix_map_to_odom from TFHelper is used to update the offset of the map and odom coordinate systems based on particle_pose and the time. The last few steps include resampling new particles based on the weights, using publish_particle_cloud for rviz, and resetting the odom movement variable to False so it is ready to sense new movement again. From here the particle filter will repeat whenever the robot moves enough to warrant a new scan.



#### RobotLocalizer
RobotLocalizer runs the show. It's the only node running, and has instances of ParticleFilter and TFHelper. As the only running node, it sets up and shares publishers for ParticleFilter. It has a running loop which executes the logic from above. In fact, reading the run function intentionally looks like a bullet pointed list of said logic.

#### ParticleFilter
ParticleFilter manages all logic and calculation for the particle filter. It isn't a node, so it gets its publishers from RobotLocalizer. It has a list of particles that is updated by the localizer when the odom frame moves enough. This involves computing weights for each particle (based on alignment between the scan at its position and the map), resampling with replacement to bias towards highly weighted particles, and move them forwards with noise to mimic the vehicle motion.

#### Particle and ParticleCloud
When initialized, the Particle class creates particles with a location, direction, and weight. Each can then be translated in its reference frame. It also has a getter for Pose.

#### OccupancyField
#### TFHelper
TFHelper was a class already given to us. We used it to help with the transforms required between the different map frames.
**---------show which transforms were used and what the final map was (like the diagram Paul drew)**

## Design Decision
When initially structuring the code we decided to have a seperate class called robot_localizer, that would instantiate the particle filter
class, as well as the occupancy field and tf_helper. The reaasoning behind this decision was to have one class that interfaces with the robot and
combines all the other classes. However, we realized later that everything in the robot localizer class could have been initialized in the particle
filter class. Essentially the robot localizer class could have been combined into a very large class with particle filter and this would have in the end
caused less disorganization.

## Challenges
+ errors / compiling everything at the end

## Future Improvements
+ better communication
+ incremental testing
+ improving computation run-time vs accuracy

## Learnings
Through the process of developing the robot localizer we got an introduction and real world application of Bayesian statistics. This is because we use a best guess of where the robot might be to determin where it actually is.
