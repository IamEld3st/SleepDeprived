import math
import numpy as np

from util.vec import Vec3
from util.orientation import *


def distance_2d(source: Vec3, destination: Vec3) -> float:
    local = destination - source
    return np.sqrt(local.x**2 + local.y**2)


def sigmoid_magic(x, strength=5):
    return 2 / (1 + np.exp(x*strength)) - 1


def find_correction(current: Vec3, ideal: Vec3) -> float:
    # Finds the angle from current to ideal vector in the xy-plane. Angle will be between -pi and +pi.

    # The in-game axes are left handed, so use -x
    current_in_radians = math.atan2(current.y, -current.x)
    ideal_in_radians = math.atan2(ideal.y, -ideal.x)

    diff = ideal_in_radians - current_in_radians

    # Make sure that diff is between -pi and +pi.
    if abs(diff) > math.pi:
        if diff < 0:
            diff += 2 * math.pi
        else:
            diff -= 2 * math.pi

    return diff
