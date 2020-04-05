import math
import numpy as np

from util.vec import Vec3
from util.orientation import *
from util.car import Car
from util.misc import *
from constants import *

from rlbot.agents.base_agent import SimpleControllerState


class MechanicStack:
    def __init__(self):
        self.stack = []
        self.target = Vec3(0, 0, 0)
        self.done = False
        self.controller = SimpleControllerState()

    def update_target(self, target):
        self.target = target
        if len(self.stack) != 0:
            self.stack[-1].update_target(self.target)

    def push(self, mechanic):
        self.stack.append(mechanic)

    def clear(self):
        self.stack = []

    def step(self, car, dt=0.0):
        if len(self.stack) == 0:
            return self.controller

        self.controller = self.stack[-1].step(car, dt)

        if self.stack[-1].done:
            self.stack.pop(-1)

        return self.controller


class BaseMechanic:
    def initialize(self):
        pass

    def __init__(self, target):
        self.target = target
        self.time_elapsed = 0.0
        self.controller = SimpleControllerState()
        self.done = False
        self.initialize()

    def update_target(self, target):
        self.target = target

    def step(self, car, dt=0.0):
        raise NotImplementedError()


class DriveMechanic(BaseMechanic):
    def step(self, car, dt=0.0):
        if not car.raw_obj.has_wheel_contact:
            pitch, roll = car.recovery()
            self.controller.pitch = pitch
            self.controller.roll = roll
        else:
            self.controller.pitch = 0.0
            self.controller.roll = 0.0

        self.controller.steer = car.steer(self.target)
        self.controller.throttle = np.clip(
            200/(self.target.z+0.1)+car.dist_2d(self.target)/1000, 0, 1)

        self.controller.boost = (not car.raw_obj.is_super_sonic and car.raw_obj.has_wheel_contact and np.abs(
            car.correction_to(self.target)) < np.pi/4 and car.dist_2d(self.target) > 1000)

        self.controller.handbrake = (car.raw_obj.has_wheel_contact and np.abs(
            car.correction_to(self.target)) > np.pi/3)

        self.done = True
        return self.controller


class HalfFlipMechanic(BaseMechanic):
    def step(self, car, dt=0.0):
        self.time_elapsed += dt
        self.controller.handbrake = True

        if self.time_elapsed < 0.1:
            self.controller.throttle = -1.0
            self.controller.steer = 0.0
        elif self.time_elapsed < 0.2:
            self.controller.pitch = 1.0
            self.controller.jump = True
        elif self.time_elapsed < 0.25:
            self.controller.pitch = 1.0
            self.controller.jump = False
        elif self.time_elapsed < 0.3:
            self.controller.pitch = 1.0
            self.controller.jump = True
        elif self.time_elapsed < 0.5:
            self.controller.jump = False
        elif self.time_elapsed < 1.0:
            self.controller.pitch = -1.0
            self.controller.roll = car.steer(self.target)
        else:
            pitch, roll = car.recovery()
            self.controller.pitch = pitch
            self.controller.roll = roll
            self.done = car.raw_obj.has_wheel_contact

        return self.controller


class DodgeMechanic(BaseMechanic):
    def step(self, car, dt=0.0):
        self.time_elapsed += dt
        pitch, yaw = car.dodge(self.target)

        if self.time_elapsed < 0.1:
            self.controller.jump = True
        elif self.time_elapsed < 0.2:
            self.controller.pitch = pitch
            self.controller.yaw = yaw
            self.controller.jump = False
        elif self.time_elapsed < 0.25:
            self.controller.pitch = pitch
            self.controller.yaw = yaw
            self.controller.jump = True
        else:
            self.controller.jump = False
            self.done = True

        return self.controller
