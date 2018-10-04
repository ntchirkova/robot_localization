#!/usr/bin/env python

from __future__ import print_function
from geometry_msgs.msg import PointStamped, PointStamped, Twist, Point
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
        #self.generate_random_points()
        while true:
            print("hi I am here")

        # if it's changed enough, send to particle filter



print('before starting')
if __name__ == '__main__':
    print('starting')
    node = RobotLocalizer()
    node.run()
