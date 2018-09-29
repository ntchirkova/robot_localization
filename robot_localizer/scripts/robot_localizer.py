#!/usr/bin/env python

from __future__ import print_function
from geometry_msgs.msg import PointStamped, PointStamped, Twist, Point
from std_msgs.msg import Header
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from datetime import datetime
from visualization_msgs.msg import Marker
import statistics
import time, numpy, math, rospy
import occupancy_field as ocf

import rospy

class RobotLocalizer(object):
    """
    doc
    """

    def __init__(self):
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        # init pf
        # subscribers and publisher

        # store array of poses and weights
        # store how it's moved ie.
        self.xs = None
        self.ys = None
        self.field = ocf.OccupancyField()
        self.particles = [] #list of particles, will be updated later


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
        width = field.map.info.width
        height = field.map.info.height
        print(width)
        print(height)
        #this does not work yet but is a a start this needs location and orientation

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
    - odom to map of robot pose
    - robot pose nearest object
    - particles nearest object
    - choose particles that might be robot - viable particles
    - generate particles around viable particles
    - move robot certain distance then stop (maybe just teleop)
    - get encoder values of robot
    - transform particles by encoder values
    -
    """


    def run(self):
        # save odom position (Odom or TF Module)
        self.generate_random_points()
        print("hi I am here")

        # if it's changed enough, send to particle filter
        pass

if __name__ == '__main__':
    node = RobotLocalizer()
    node.run()
