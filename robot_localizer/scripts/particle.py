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
        self.angle = self.tfHelper.angle_normalize(self.theta + angle)

class ParticleCloud(object):

    def __init__(self, p):
        self.particle = p
        self.pts = []

    def generate_points(self, robo_pts):
        l = len(robo_pts)
        for i in range(l):
            dif_ang = 2*math.pi/l * i
            x = robo_pts[i] * math.cos(self.particle.theta + dif_ang)
            y = robo_pts[i] * math.sin(self.particle.theta + dif_ang)
            self.pts.append((x,y))
