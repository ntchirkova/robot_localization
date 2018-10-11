#!/usr/bin/env python

""" This is the starter code for the robot localization project """

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose

import numpy as np
from particle import Particle, ParticleCloud
from helper_functions import TFHelper
from occupancy_field import OccupancyField
from visualization_msgs.msg import MarkerArray
import random as r
import math


class ParticleFilter(object):
    """ The class that represents a Particle Filter ROS Node
    """
    def __init__(self):
        # # pose_listener responds to selection of a new approximate robot
        # # location (for instance using rviz)
        # rospy.Subscriber("initialpose",
        #                  PoseWithCovarianceStamped,
        #                  self.update_initial_pose)

        # create instances of two helper objects that are provided to you
        # as part of the project
        self.occupancy_field = OccupancyField()
        self.transform_helper = TFHelper()
        self.particles = []

    def publish_particle_cloud(self, publisher):
        # Make pose from particle for all particles
        particle_poses = [particle.get_pose() for particle in self.particles]

        # Publish
        publisher.publish(particle_poses)

    def publish_top_particle(self, publisher):
        top_particle = self.particles[0]

        for particle in self.particles:
            if particle.weight > top_particle.weight:
                top_particle = particle

        print([top_particle.get_pose()])
        publisher.publish([top_particle.get_pose()])

    def gen_init_particles(self):
        """Generating random particles with x, y, and t values"""
        width = self.occupancy_field.map.info.width
        height = self.occupancy_field.map.info.height
        for i in range(500):
            x = r.randrange(0,width)
            y = r.randrange(0,height)
            t = math.radians(r.randrange(0,360))
            p = Particle(x,y,t)
            self.particles.append(p)

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter
            based on a pose estimate.  These pose estimates could be generated
            by another ROS Node or could come from the rviz GUI """
        xy_theta = \
            self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)

        # TODO this should be deleted before posting
        self.transform_helper.fix_map_to_odom_transform(msg.pose.pose,
                                                        msg.header.stamp)
        # initialize your particle filter based on the xy_theta tuple

    def resample_particles(self):
        """Resample particles with replacement."""
        if len(self.particles):
            weights = [particle.weight for particle in self.particles]

            return list(np.random.choice(
                self.particles,
                size=len(self.particles),
                replace=True,
                p=weights,
            ))
        else:
            print("No particles to resample from")
            return None

    def update_all_particles(self, transform):
        for particle in self.particles:
            self.update_particle_with_randomness(particle, transform)

    def update_particle_with_randomness(self, particle, transform):
        # TODO(matt): Make this a tunable param
        DISTANCE_VAR_SCALE = 0.1
        ANGLE_VAR_SCALE = math.radians(5)

        # NOTE: We scale the variance instead of the standard deviation because
        # that makes it independent of the update time (the noise in one update
        # will be the same as the sum of the noise in two updates)
        distance = math.sqrt(transform['translation'][0]**2 + transform['translation'][1]**2)
        translation_mean, translation_var = 0, DISTANCE_VAR_SCALE * distance  # scale with magnitude
        rotation_mean, rotation_var = 0, ANGLE_VAR_SCALE

        modified_transform = transform
        modified_transform['translation'][0] += np.random.normal(translation_mean, math.sqrt(translation_var), 1)
        modified_transform['translation'][1] += np.random.normal(translation_mean, math.sqrt(translation_var), 1)
        modified_transform['rotation'] += np.random.normal(rotation_mean, math.sqrt(rotation_var))

        self.update_particle(particle, modified_transform)

    def update_particle(self, particle, transform):
        # rotate translation in the particle's direction
        particle_translation = self.transform_helper.rotate_2d_vector(transform['translation'], particle.theta)

        particle.translate(particle_translation)
        particle.rotate(transform['rotation'])


    # def compare_points(self):
    #     """Compares translated particle to lidar scans, returns weights values"""
    #     distances = []
    #     errordis = 0
    #     for a in range(500):
    #         particle.ParticleCloud(self.particle[a])
    #         for b in range(8):
    #             d[b] = OccupancyField.get_closest_obstacle_distance(particle.ParticleCloud[b][1],particle.ParticleCloud[b][2])
    #         particle.Particle.weight = 1 / (sum(d) + .01)

    def compare_points(self, robo_pts):
        """ This function determines the weights for each particle.

        Args:
            robo_pts (list): is a list of lidar readings for n directions. It can
                             be obtained by calling get_x_directions in robot localizerself.
        """
        for p in self.particles:
            p_cloud = ParticleCloud(p)
            p_cloud.generate_points(robo_pts)
            d = []
            for pt in p_cloud.pts:
                d.append(self.occupancy_field.get_closest_obstacle_distance(pt[0],pt[1]))
            p.weight = 1 / (sum(d) + .01)



    def run(self):
        r = rospy.Rate(5)

        while not(rospy.is_shutdown()):
            # in the main loop all we do is continuously broadcast the latest
            # map to odom transform
            self.transform_helper.send_last_map_to_odom_transform()
            r.sleep()


# use tf module to get transform between last pos and current pos, and apply relative transform to particles.

if __name__ == '__main__':
    n = ParticleFilter()
    n.run()
