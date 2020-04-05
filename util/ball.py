import math
import numpy as np

from util.vec import Vec3
from util.orientation import *
from util.misc import *


class Car:
    def __init__(self, ball_obj):
        self.loc = Vec3(ball_obj.physics.location)
        self.vel = Vec3(ball_obj.physics.velocity)
        self.rot = Vec3(ball_obj.physics.rotation)
        self.raw_obj = ball_obj

    def dist_2d(self, target):
        return distance_2d(self.loc, target)

