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
import occupancy_field

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
        self.field = occupancy_field.OccupancyField()


    def process_scan(self, m):
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

    def generate_random_points(self):
        width = field.map.info.width
        height = field.map.info.height
        print(width)
        print(height)


    def run(self):
        # save odom position (Odom or TF Module)
        self.generate_random_points()
        print("hi I am here")

        # if it's changed enough, send to particle filter
        pass

if __name__ == '__main__':
    node = RobotLocalizer()
    node.run()
