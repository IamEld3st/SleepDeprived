from util.orientation import Orientation, relative_location
from util.vec import Vec3
import math
import numpy as np
from rlbot.agents.base_agent import SimpleControllerState

from mechanics import *


class BaseState:
    def initialize(self):
        pass

    def __init__(self):
        self.expired = False
        self.mechanic_stack = MechanicStack()
        self.time_elapsed = 0.0
        self.previous_tick_time = 0.0
        self.controller = SimpleControllerState()
        self.initialize()

    def update(self, packet, ball_pred, field_info, ind):
        raise NotImplementedError()


class EmptyState(BaseState):
    def update(self, packet, ball_pred, field_info, ind):
        self.expired = True
        return self.controller


class GoToBallState(BaseState):
    def initialize(self):
        self.last_dodge_time = 0.0

    def update(self, packet, ball_pred, field_info, ind):
        ball_loc = Vec3(packet.game_ball.physics.location)
        my_car = Car(packet.game_cars[ind])
        dt = packet.game_info.seconds_elapsed - self.previous_tick_time

        if len(self.mechanic_stack.stack) == 0:
            self.mechanic_stack.push(DriveMechanic(ball_loc))

        if not len(self.mechanic_stack.stack) > 1:
            if np.abs(my_car.correction_to(ball_loc)) > np.pi/1.1:
                self.mechanic_stack.push(HalfFlipMechanic(ball_loc))
            elif my_car.dist_2d(ball_loc) < 600 and packet.game_info.seconds_elapsed - self.last_dodge_time > 4.0:
                self.mechanic_stack.push(DodgeMechanic(ball_loc))

        self.mechanic_stack.update_target(ball_loc)

        self.expired = my_car.dist_2d(ball_loc) > 2500

        self.previous_tick_time = packet.game_info.seconds_elapsed
        return self.mechanic_stack.step(my_car, dt)


# class TakeShot(BaseState):


# class GoToDefensiveState(BaseState):


# class DefenseState(BaseState):


class GrabBoostState(BaseState):
    def initialize(self):
        self.boost_aquired = False
        self.boost_index = -1
        self.boost_loc = None

    def update(self, packet, ball_pred, field_info, ind):
        my_car = Car(packet.game_cars[ind])
        if len(self.mechanic_stack.stack) == 0:
            self.mechanic_stack.push(DriveMechanic(Vec3(0.0, 0.0, 0.0)))

        if (not self.boost_aquired) and my_car.loc.z < 100:
            big_boosts_dist = 99999

            for i in range(field_info.num_boosts):
                if field_info.boost_pads[i].is_full_boost and packet.game_boosts[i].is_active:
                    location = Vec3(field_info.boost_pads[i].location)
                    if my_car.dist_2d(location) < big_boosts_dist:
                        big_boosts_dist = my_car.dist_2d(location)
                        self.boost_loc = location
                        self.mechanic_stack.update_target(location)
                        self.boost_index = i

            self.boost_aquired = True
        elif not packet.game_boosts[self.boost_index].is_active:
            self.boost_aquired = False
        
        self.mechanic_stack.update_target(self.boost_loc)

        self.expired = my_car.raw_obj.boost > 80

        return self.mechanic_stack.step(my_car)


class GoOutOfGoalState(BaseState):
    def update(self, packet, ball_pred, field_info, ind):
        my_car = Car(packet.game_cars[ind])
        target_loc = Vec3(my_car.loc.x, my_car.loc.y * .9, my_car.loc.z)
        if len(self.mechanic_stack.stack) == 0:
            self.mechanic_stack.push(DriveMechanic(target_loc))

        self.mechanic_stack.update_target(target_loc)

        self.expired = np.abs(my_car.loc.y) < 5120

        return self.mechanic_stack.step(my_car)


class GetOffTheWallState(BaseState):
    def update(self, packet, ball_pred, field_info, ind):
        my_car = Car(packet.game_cars[ind])
        target_loc = Vec3(my_car.loc.x, my_car.loc.y, 0.0)

        if len(self.mechanic_stack.stack) == 0:
            self.mechanic_stack.push(DriveMechanic(target_loc))

        self.mechanic_stack.update_target(target_loc)


        self.expired = my_car.loc.z < 200

        return self.mechanic_stack.step(my_car)


class KickOffState(BaseState):
    def update(self, packet, ball_pred, field_info, ind):
        my_car = Car(packet.game_cars[ind])
        ball_loc = Vec3(packet.game_ball.physics.location)
        dt = packet.game_info.seconds_elapsed - self.previous_tick_time

        if self.mechanic == None:
            self.mechanic = DriveMechanic(ball_loc)

        if len(self.persist) < 3:
            self.persist = [my_car.loc, my_car.raw_obj.boost, 0]

        if self.mechanic.done:
            if self.persist[2] == 0:
                if self.persist[0].x < 150.0:
                    target_loc = Vec3(500, 0, 0)
                elif np.abs(self.persist[0].y) > 3000:
                    target_loc = Vec3(200*-np.sign(self.persist[0].x), 0, 0)
                else:
                    target_loc = Vec3(0, my_car.loc.y*2.65, 0)
                self.mechanic = DriveMechanic(target_loc)
                if self.persist[1] < my_car.raw_obj.boost:
                    self.persist[2] = 1
            elif self.persist[2] == 1:
                self.mechanic = DodgeMechanic(ball_loc)
                self.persist[2] = 2
            else:
                self.mechanic = DriveMechanic(ball_loc)

        self.expired = bool(ball_loc.x + ball_loc.y)

        self.persist[1] = my_car.raw_obj.boost
        self.previous_tick_time = packet.game_info.seconds_elapsed
        return self.mechanic.step(my_car, dt)


class TestState(BaseState):
    def update(self, packet, ball_pred, field_info, ind):
        my_car = Car(packet.game_cars[ind])
        dt = packet.game_info.seconds_elapsed - self.previous_tick_time

        self.time_elapsed += dt
        if self.mechanic == None:
            self.mechanic = SpeedFlipMechanic(my_car.loc)

        if self.time_elapsed > 5 and self.mechanic.done:
            self.time_elapsed = 0.0
            self.mechanic = SpeedFlipMechanic(my_car.loc)

        self.previous_tick_time = packet.game_info.seconds_elapsed
        return self.mechanic.step(my_car, dt)
