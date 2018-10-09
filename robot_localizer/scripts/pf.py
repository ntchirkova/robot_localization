#!/usr/bin/env python

""" This is the starter code for the robot localization project """

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose

import numpy as np
from particle import Particle
from helper_functions import TFHelper
from occupancy_field import OccupancyField
import random as r
import math


class ParticleFilter(object):
    """ The class that represents a Particle Filter ROS Node
    """
    def __init__(self):
        rospy.init_node('pf')

        # pose_listener responds to selection of a new approximate robot
        # location (for instance using rviz)
        rospy.Subscriber("initialpose",
                         PoseWithCovarianceStamped,
                         self.update_initial_pose)

        # publisher for the particle cloud for visualizing in rviz.
        self.particle_pub = rospy.Publisher("particlecloud",
                                            PoseArray,
                                            queue_size=10)

        #publisher for the top weighted particle
        self.topparticle_pub = rospy.Publisher("topparticle",
                                            PoseArray,
                                            queue_size=10)

        # create instances of two helper objects that are provided to you
        # as part of the project
        self.occupancy_field = OccupancyField()
        self.transform_helper = TFHelper()
        self.particles = []

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

    def update_particle_with_randomness(self, particle, transform):
        # TODO(matt): Make this a tunable param
        DISTANCE_VAR_SCALE = 0.1
        ANGLE_VAR_SCALE = math.radians(5)

        # NOTE: We scale the variance instead of the standard deviation because
        # that makes it independent of the update time (the noise in one update
        # will be the same as the sum of the noise in two updates)
        distance = math.sqrt(transform.translation[0]**2 + transform.translation[1]**2)
        translation_mean, translation_var = 0, DISTANCE_VAR_SCALE * distance  # scale with magnitude
        rotation_mean, rotation_var = 0, ANGLE_VAR_SCALE

        modified_transform = transform
        modified_transform.translation[0] += np.random.normal(translation_mean, math.sqrt(translation_var), 1)
        modified_transform.translation[1] += np.random.normal(translation_mean, math.sqrt(translation_var), 1)
        modified_transform.rotation += np.random.normal(rotation_mean, math.sqrt(rotation_var))

        self.update_particle(particle, modified_transform)

    def update_particle(self, particle, transform):
        # rotate translation in the particle's direction
        particle_translation = self.transform_helper.rotate_2d_vector(transform.translation, particle.angle)

        particle.translate(particle_translation)
        particle.rotate(transform.rotation)

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
