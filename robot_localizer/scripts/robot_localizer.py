#!/usr/bin/env python

from __future__ import print_function
from geometry_msgs.msg import PointStamped, PointStamped, Twist, Point, PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
from particle import Particle
from particle import ParticleCloud
import matplotlib.pyplot as plt
from datetime import datetime
from visualization_msgs.msg import Marker
import statistics
import random as r
import time, numpy, math, rospy
from helper_functions import TFHelper
from occupancy_field import OccupancyField
from pf import ParticleFilter

import rospy

class RobotLocalizer(object):
    """
    doc
    """

    def __init__(self):
        print("init RobotLocalizer")
        rospy.init_node('localizer')
        self.tfHelper = TFHelper()

        self.particle_filter = ParticleFilter()
        print("ParticleFilter initialized")
        self.xs = None
        self.ys = None
        self.ranges = []  # Lidar scan

        self.last_odom_msg = None
        print(self.last_odom_msg)
        self.diff_transform = {
                'translation': None,
                'rotation': None,
            }

        self.odom_changed = False # Toggles to True when the odom frame has changed enough

        # subscribers and publisher
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.update_odom)

        ### Used for the particle filter
        # publisher for the particle cloud for visualizing in rviz.
        self.particle_pub = rospy.Publisher("particlecloud",
                                            PoseArray,
                                            queue_size=10)
        # publisher for the top weighted particle
        self.topparticle_pub = rospy.Publisher("topparticle",
                                            PoseArray,
                                            queue_size=10)

        print("RobotLocalizer initialized")

    def update_odom(self, msg):
        MIN_TRAVEL_DISANCE = 0.1
        MIN_TRAVEL_ANGLE = math.radians(5)

        if self.last_odom_msg is None:
            self.last_odom_msg = msg
            return

        last_xyt = self.tfHelper.convert_pose_to_xy_and_theta(self.last_odom_msg.pose.pose)
        current_xyt = self.tfHelper.convert_pose_to_xy_and_theta(msg.pose.pose)

        # Get translation in odom
        translation = [
            current_xyt[0] - last_xyt[0],  # x
            current_xyt[1] - last_xyt[1],  # y
        ]

        # rotate to vehicle frame
        translation = self.tfHelper.rotate_2d_vector(translation, last_xyt[2])

        # get orientation diff
        theta = self.tfHelper.angle_diff(current_xyt[2], last_xyt[2])

        # Schedule to update particle filter if there's enough change
        distance_travelled = math.sqrt(translation[0] ** 2 + translation[1] ** 2)
        print(distance_travelled)
        if distance_travelled > MIN_TRAVEL_DISANCE or theta > MIN_TRAVEL_ANGLE:
            # TODO(matt): consider using actual transform
            # last_to_current_transform = self.tfHelper.convert_translation_rotation_to_pose(
            #     translation, self.tfHelper.convert_theta_to_quaternion(theta)
            # )

            last_to_current_transform = {
                'translation': translation,
                'rotation': theta,
            }

            self.diff_transform = last_to_current_transform
            self.last_odom_msg = msg
            self.odom_changed = True


    def process_scan(self, m):
        """Storing lidar data
        """
        #TODO:
        self.ranges = m.ranges
        xs = []
        ys = []
        for i in range(len(self.ranges)):
            if self.ranges[i] != 0:
                theta = math.radians(i)
                r = self.ranges[i]
                xf = math.cos(theta)*r
                yf = math.sin(theta)*r
                xs.append(xf)
                ys.append(yf)

        self.xs = xs
        self.ys = ys

    def get_x_directions(self, x):
        interval = 360/x
        angle = 0
        directions = []
        for i in range(x):
            dist = self.ranges[angle]
            directions.append((math.radians(angle),dist))
            angle = angle + interval
        return directions

    def gen_neighbor_particles(self):
        """Generates particles around given points"""
        #TODO:
        pass

    def find_all_nearest_objects(self):
        """Determines nearest non-zero point of all point (loops through)"""
        #TODO:
        pass

    def get_encoder_value(self):
        """Records odom movement, translate to x, y, and theta"""
        #TODO:
        pass



    """
    Functions to write or figure out where they are:
    Order of particle filter:

    1. DONE generate initial 500 random particles
    2. DONE get ranges from robot
        -determine 8 values for directions
        -find lowest distance to obstacle
    3. Process particles
     - project lowest distance from robot onto each particle
        -DONE for each particle get nearest object -> error distance
        -DONE 1/error distance = particle.weight
    4. DONE publish particle with highest weight
    5. DONE resample particles based on weight
    6. DONE move robot - get transform
    7. DONE transform resampled points with randomness

    """


    def run(self):
        # save odom position (Odom or TF Module)
        # self.generate_random_points()
        NUM_DIRECTIONS = 8
        self.particle_filter.gen_init_particles()
        # # Get lidar readings in x directions
        # robo_pts = self.get_x_directions(NUM_DIRECTIONS)
        # # For each particle compare lidar scan with map
        # self.particle_filter.compare_points(robo_pts)
        #
        # # Publish best guessself.particle_filter.gen_init_particles()
        # self.particle_filter.publish_top_particle(self.topparticle_pub)
        #
        # # Resample particles
        # self.particle_filter.resample_particles()
        #
        # # Publish cloud
        # self.particle_filter.publish_particle_cloud(self.particle_pub)

        while not(rospy.is_shutdown()):
            if (self.odom_changed):
                print("Odom changed, let's do some stuff")

                # Get lidar readings in x directions
                robo_pts = self.get_x_directions(NUM_DIRECTIONS)

                # Update particles
                self.particle_filter.update_all_particles(self.diff_transform)

                # For each particle compare lidar scan with map
                self.particle_filter.compare_points(robo_pts)

                # Publish best guessself.particle_filter.gen_init_particles()
                self.particle_filter.publish_top_particle(self.topparticle_pub)

                # Resample particles
                self.particle_filter.resample_particles()

                # Publish cloud
                self.particle_filter.publish_particle_cloud(self.particle_pub)

                # Wait until robot moves enough again
                self.odom_changed = False


print('before starting')
if __name__ == '__main__':
    print('starting')
    node = RobotLocalizer()
    node.run()
