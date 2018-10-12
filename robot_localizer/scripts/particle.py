from __future__ import print_function, division

from helper_functions import TFHelper
import math

class Particle(object):

    def __init__(self,x,y,t):
        self.x = x
        self.y = y
        self.theta = t
        self.weight = 1

        self.tfHelper = TFHelper()

    def translate(self, translation):
        self.x += translation[0]
        self.y += translation[1]

    def rotate(self, angle):
        self.theta = self.tfHelper.angle_normalize(self.theta + angle)

    def get_pose(self):
        translation = [self.x, self.y, 0]
        rotation = self.tfHelper.convert_theta_to_quaternion(self.theta)
        return self.tfHelper.convert_translation_rotation_to_pose(translation, rotation)

class ParticleCloud(object):

    def __init__(self, p):
        self.particle = p
        self.pts = []

    def generate_points(self, robo_pts):
        robo_pts_len = len(robo_pts)
        for i in range(robo_pts_len):
            diff_ang = 2*math.pi/robo_pts_len * i
            x = robo_pts[i][0] * math.cos(self.particle.theta + diff_ang)
            y = robo_pts[i][1] * math.sin(self.particle.theta + diff_ang)
            self.pts.append((x,y))
