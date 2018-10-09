#!/usr/bin/env python

from __future__ import print_function
from geometry_msgs.msg import PointStamped, PointStamped, Twist, Point
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
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        # init pf
        # subscribers and publisher
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.update_odom)

        self.tfHelper = TFHelper()

        self.particle_filter = ParticleFilter()

        self.xs = None
        self.ys = None

        self.last_odom_msg = None
        self.diff_transform = {
                'translation': None,
                'rotation': None,
            }

        self.odom_changed = False # Toggles to True when the odom frame has changed enough

    def update_odom(self, msg):
        MIN_TRAVEL_DISANCE = 0.25
        MIN_TRAVEL_ANGLE = math.radians(10)

        last_xyt = self.tfHelper.convert_pose_to_xy_and_theta(self.last_odom_msg.pose)
        current_xyt = self.tfHelper.convert_pose_to_xy_and_theta(msg.pose)

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
        ranges = m.ranges
        xs = []
        ys = []
        for i in range(len(self.ranges)):
            if self.ranges[i] != 0:
                theta = math.radians(i)
                r = ranges[i]
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

<<<<<<< HEAD
    def compare_points(self):
        """Compares translated particle to lidar scans, returns weights values"""
        d = []
        errordis = 0
        for a in range(500):
            particle.ParticleCloud(self.particle[a])
            for b in range(8):
                d[b] = OccupancyField.get_closest_obstacle_distance(particle.ParticleCloud[b][1],particle.ParticleCloud[b][2])
            particle.Particle.weight = 1 / (sum(d) + .01)
=======
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

>>>>>>> 7695098717f8cefa7112d4e251a50909e5f9c5aa


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
    4. publish particle with highest weight
    5. DONE resample particles based on weight
    6. DONE move robot - get transform
    7. DONE transform resampled points with randomness

    """


    def run(self):
        # save odom position (Odom or TF Module)
        # self.generate_random_points()
        NUM_DIRECTIONS = 8

        if (self.odom_changed):
            # Get lidar readings in every direction
            self.get_x_directions(NUM_DIRECTIONS)

            # For each particle compare lidar scan with map
            self.particle_filter.compare_points()

            # Publish best guess

            # Resample particles
            self.particle_filter.resample_particles()

            # Update particles
            self.particle_filter.update_all_particles(self.diff_transform)

            # Wait until robot moves enough again
            self.odom_changed = False


print('before starting')
if __name__ == '__main__':
    print('starting')
    node = RobotLocalizer()
    node.run()
