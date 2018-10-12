from __future__ import print_function, division

from helper_functions import TFHelper
import math

class Particle(object):

    def __init__(self,x,y,t, w=1):
        self.x = x
        self.y = y
        self.theta = t
        self.weight = w
        
    def translate(self, translation):
        self.x += translation[0]
        self.y += translation[1]

    def rotate(self, angle):
        self.theta = TFHelper.angle_normalize(self.theta + angle)

    def get_pose(self):
        translation = [self.x, self.y, 0]
        rotation = TFHelper.convert_theta_to_quaternion(self.theta)
        return TFHelper.convert_translation_rotation_to_pose(translation, rotation)

    def deep_copy(self):
        return Particle(self.x, self.y, self.theta, self.weight)

class ParticleCloud(object):

    def __init__(self, p):
        self.particle = p
        self.pts = []

    def generate_points(self, robo_pts):
        # robo_pts: [[angle, distance]]
        robo_pts_len = len(robo_pts)
        for i in range(robo_pts_len):
            diff_ang = robo_pts[i][0]
            x = self.particle.x + robo_pts[i][1] * math.cos(self.particle.theta + diff_ang)
            y = self.particle.y + robo_pts[i][1] * math.sin(self.particle.theta + diff_ang)
            self.pts.append((x,y))
