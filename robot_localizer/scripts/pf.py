#!/usr/bin/env python

""" This is the starter code for the robot localization project """

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose

import numpy as np
from particle import Particle, ParticleCloud
from helper_functions import TFHelper
from occupancy_field import OccupancyField
from visualization_msgs.msg import Marker, MarkerArray
import random as r
import math, time
from copy import deepcopy


class ParticleFilter(object):
    """ The class that represents a Particle Filter ROS Node
    """
    def __init__(self, top_particle_pub, particle_cloud_pub, particle_cloud_marker_pub):
        # # pose_listener responds to selection of a new approximate robot
        # # location (for instance using rviz)
        # rospy.Subscriber("initialpose",
        #                  PoseWithCovarianceStamped,
        #                  self.update_initial_pose)

        self.top_particle_pub = top_particle_pub
        self.particle_cloud_pub = particle_cloud_pub
        self.particle_cloud_marker_pub = particle_cloud_marker_pub

        # create instances of two helper objects that are provided to you
        # as part of the project
        self.occupancy_field = OccupancyField()
        self.transform_helper = TFHelper()
        self.particles = []
        self.markerArray = MarkerArray()

    def gauge_particle_position(self):
        xs = [particle.x for particle in self.particles]
        ys = [particle.y for particle in self.particles]

        print("min x: {} --- average x: {} --- max x: {} \nmin y: {} --- average y: {} --- max y: {}".format(
            min(xs), sum(xs)/len(xs), max(xs), min(ys), sum(ys)/len(ys), max(ys) 
        ))

    def get_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = marker.SPHERE
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        return marker

    def draw_markerArray(self):
        markerArray = MarkerArray()
        for p in self.particles:
            m = self.get_marker(p.x, p.y)
            markerArray.markers.append(m)
        self.particle_cloud_marker_pub.publish(markerArray)

    def publish_particle_cloud(self):
        msg = PoseArray()
        msg.header.frame_id = "map"

        # Make pose from particle for all particles
        msg.poses = [particle.get_pose() for particle in self.particles]

        # Publish
        self.particle_cloud_pub.publish(msg)

    def publish_top_particle(self):
        msg = PoseArray()

        top_particle = self.particles[0]

        for particle in self.particles:
            if particle.weight > top_particle.weight:
                top_particle = particle

        msg.poses.append(top_particle.get_pose())
        # print(msg)
        self.top_particle_pub.publish(msg)

        return top_particle.get_pose()

    def gen_init_particles(self):
        """Generating random particles with x, y, and t values"""
        # width = self.occupancy_field.map.info.width
        # height = self.occupancy_field.map.info.height
        width = 10
        height = 10
        print("map width: {}, height: {}".format(width, height))
        for i in range(500):
            # x = r.randrange(0,width)
            # y = r.randrange(0,height)
            x = r.uniform(-5, 5)
            y = r.uniform(-5, 5)
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
            weights = [particle.weight  if not math.isnan(particle.weight) else 0.0001 for particle in self.particles]
            total_weight = sum(weights)
            weights = [weight / total_weight for weight in weights]

            before = time.time()
            self.particles = [particle.deep_copy() for particle in list(np.random.choice(
            # self.particles = [particle for particle in list(np.random.choice(
                self.particles,
                size=len(self.particles),
                replace=True,
                p=weights,
            ))]
            after = time.time()
            print("************************************timer: {}".format(after-before))
            print("number of particles: {}".format(len(self.particles)))
        else:
            print("No particles to resample from")
            return None

    def update_all_particles(self, transform):
        for particle in self.particles:
            # self.update_particle(particle, transform)
            self.update_particle_with_randomness(particle, transform)

    def update_particle_with_randomness(self, particle, transform):
        # TODO(matt): Make this a tunable param
        DISTANCE_VAR_SCALE = 0.001
        ANGLE_VAR_SCALE = math.radians(0.5)

        # NOTE: We scale the variance instead of the standard deviation because
        # that makes it independent of the update time (the noise in one update
        # will be the same as the sum of the noise in two updates)
        distance = math.sqrt(transform['translation'][0]**2 + transform['translation'][1]**2)
        translation_mean, translation_var = 0, DISTANCE_VAR_SCALE #* distance  # scale with magnitude
        rotation_mean, rotation_var = 0, ANGLE_VAR_SCALE

        modified_transform = transform
        # modified_transform['translation'][0] += float(np.random.normal(translation_mean, math.sqrt(translation_var), 1))
        # modified_transform['translation'][1] += float(np.random.normal(translation_mean, math.sqrt(translation_var), 1))
        # modified_transform['rotation'] += float(np.random.normal(rotation_mean, math.sqrt(rotation_var)))

        modified_transform['translation'][0] += float(np.random.uniform(-0.01, 0.01, 1))
        modified_transform['translation'][1] += float(np.random.uniform(-0.01, 0.01, 1))
        modified_transform['rotation'] += float(np.random.uniform(-math.radians(5), -math.radians(5), 1))

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
                if pt[1] != 0:
                    d dist = abs(self.occupancy_field.get_closest_obstacle_distance(pt[0],pt[1]))
                    d.append(math.exp(-d**2/1))
            p.weight = 1 / (sum(d) + .01)

    def run(self):
        pass
        # r = rospy.Rate(5)

        # while not(rospy.is_shutdown()):
        #     # in the main loop all we do is continuously broadcast the latest
        #     # map to odom transform
        #     self.transform_helper.send_last_map_to_odom_transform()
        #     r.sleep()


# use tf module to get transform between last pos and current pos, and apply relative transform to particles.

if __name__ == '__main__':
    pass
    # n = ParticleFilter()
    # n.run()
