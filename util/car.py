import math
import numpy as np

from util.vec import Vec3
from util.orientation import *
from util.misc import *


class Car:
    def __init__(self, car_obj):
        self.loc = Vec3(car_obj.physics.location)
        self.vel = Vec3(car_obj.physics.velocity)
        self.rot = car_obj.physics.rotation
        self.ori = Orientation(self.rot)
        self.raw_obj = car_obj

    def correction_to(self, target):
        rel_loc = relative_location(self.loc, self.ori, target)
        return math.atan2(-rel_loc.y, rel_loc.x)

    def rel_loc(self, target):
        return relative_location(self.loc, self.ori, target)

    def dist_2d(self, target):
        return distance_2d(self.loc, target)

    def steer(self, correction: float or Vec3):
        if type(correction) == Vec3:
            correction = self.correction_to(correction)
        return sigmoid_magic(correction)

    def recovery(self):
        return sigmoid_magic(self.ori.pitch, 1), sigmoid_magic(self.ori.roll, 1)

    def dodge(self, target):
        dir_loc = self.rel_loc(target).normalized()
        '''
        vf = self.vel.dot(self.ori.forward)
        s = np.abs(vf) / 2300.0

        if np.abs(vf) < 100.0:
            backward_dodge = dir_loc.x < 0.0
        else:
            backward_dodge = (dir_loc.x >= 0.0) != (vf > 0.0)

        pitch = dir_loc.x / ((16.0 / 15.0) * (1.0 + 1.5 * s)
                             ) if backward_dodge else dir_loc.x / 1
        yaw = dir_loc.y / (1.0 + 0.9 * s)

        temp = Vec3(pitch, yaw, 0).normalized()
        # Pitch, Yaw
        return -temp.x, temp.y
        '''
        return -dir_loc.x, dir_loc.y
    
    def get_ball_avoid(self, ball_loc):
        pass
