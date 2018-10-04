#!/usr/bin/env python

from __future__ import print_function
from geometry_msgs.msg import PointStamped, PointStamped, Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
from particle import Particle
import matplotlib.pyplot as plt
from datetime import datetime
from visualization_msgs.msg import Marker
import statistics
import random as r
import time, numpy, math, rospy
import occupancy_field as ocf
from helper_functions import TFHelper

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

        # store array of poses and weights
        # store how it's moved ie.
        self.xs = None
        self.ys = None
        self.field = ocf.OccupancyField()

        # TODO: Should this be in the particle filter?
        self.particles = [] #list of particles, will be updated later

        self.last_odom_msg = None
        self.diff_transform = last_to_current_transform = {
                'translation': None,
                'rotation': None,
            }
        self.odom_changed = False # Toggles to True when 


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
        ranges = m.ranges
        xs = []
        ys = []
        xsf = []
        ysf = []
        for i in range(len(ranges)):
            if ranges[i] != 0:
                theta = math.radians(i)
                r = ranges[i]
                xf = math.cos(theta)*r
                yf = math.sin(theta)*r
                xs.append(xf)
                ys.append(yf)

        self.xs = xs
        self.ys = ys

    def gen_init_particles(self):
        """Generating random particles with x, y, and t values"""
        #TODO: test width and height, generate random particles
        width = self.field.map.info.width
        height = self.field.map.info.height
        print(width)
        print(height)
        for i in range(500):
            x = r.randrange(0,width)
            y = r.randrange(0,height)
            t = math.radians(r.randrange(0,360))
            p = Particle(x,y,t)
            self.particles.append(p)

        #this does not work yet but is a a start this needs location and orientation, should work with map

    def gen_neighbor_particles(self):
        """Generates particles around given points"""
        #TODO:
        pass

    def find_all_nearest_objects(self):
        """Determines nearest non-zero point of all point (loops through)"""
        #TODO:
        pass

    def compare_points(self):
        """Compares particles to lidar scans, returns weights / probablility values"""
        #TODO:
        pass

    def teleop(self):
        """Adds teleop functionality, records encoder values"""
        #TODO:
        pass

    def get_encoder_value(self):
        """Records odom movement, translate to x, y, and theta"""
        #TODO:
        pass



    """
    Functions to write or figure out where they are:
    Order of particle filter:

    1. generate initial 500 random particles
    2. get ranges from robot
        - determine 8 values for directions
    3. Process particles
     - project 8 distance from robot onto each particle -> gives a list of 8 points
        - for each of 8 points of particle get nearest object -> sums to error distance
        - 1/error distance = particle.weight
    4. publish particle with highest weight
    5. resample particles based on weight
    6. move robot - get transform
    7. transform resampled points with randomness

    """


    def run(self):
        # save odom position (Odom or TF Module)
        # self.generate_random_points()

        # For testing
        while True:
            print("hi I am here")

        if (self.odom_changed):
            pass # Do the particle filter stuff

            self.odom_changed = False
        pass


print('before starting')
if __name__ == '__main__':
    print('starting')
    node = RobotLocalizer()
    node.run()
