from __future__ import print_function, division

from helper_functions import TFHelper

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
