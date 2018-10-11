# Robot Localization

Siena Okuno, Matthew Beaudouin, Nina Tchirkova

10/12/18
Computational Robotics

Olin College of Engineering

## Project Goal
The objective of this project was to create a working robot localization algorithm.

## Implementation Overview
Using ros-based processes, we implemented a particle filter node that contained 5 main object-oriented scripts. Within each of these were the different classes:TFHelper, Particle, ParticleCloud, OccupancyField, ParticleFilter, and RobotLocalizer. These classes will be explained in detail later.

**---------setup before localization: bag map, set up rviz**

A brief overview of our particle filter run-time:
+ Initialize 500 random particles with direction and 2D location
+ Get lidar scan
  + Get ranges (a dictionary of angles *a* and distances *d*) from lidar
  + Filter down to 8 non-zero ranges spaced evenly from 0 to 2pi
+ For each particle, generate point at the 8 angles *d* distance away from the original particle
+ Determine the closest object to each projected point and compute error distance
+ Calculate weight (or probability that particle is correct) using the error distance
+ Publish particle with highest weight
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
