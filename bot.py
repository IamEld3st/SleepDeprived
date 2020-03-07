import math
import numpy as np

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket

from util.orientation import Orientation, relative_location
from util.vec import Vec3
from states import *


class MyBot(BaseAgent):

    def initialize_agent(self):
        self.controller_state = SimpleControllerState()
        self.last_score_total = 0
        self.overtime_val_change = False
        self.state = KickOffState()

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        my_car = packet.game_cars[self.index]
        car_location = Vec3(my_car.physics.location)
        ball_location = Vec3(packet.game_ball.physics.location)
        fieldinfo = self.get_field_info()
        ballprediction = self.get_ball_prediction_struct()

        if not self.state.expired:
            self.controller_state = self.state.update(
                packet, ballprediction, fieldinfo, self.index)

            # Safety states
            if car_location.z > 300 and my_car.has_wheel_contact:
                self.state = GetOffTheWallState()
            if np.abs(car_location.y) > 5120:  # Inside a goal
                self.state = GoOutOfGoalState()
            if (packet.teams[0].score + packet.teams[1].score > self.last_score_total and packet.game_info.is_kickoff_pause) or packet.game_info.is_overtime != self.overtime_val_change:
                self.last_score_total = packet.teams[0].score + \
                    packet.teams[1].score
                self.overtime_val_change = packet.game_info.is_overtime
                self.state = KickOffState()
        else:
            if distance_2d(car_location, ball_location) < 2500:
                self.state = GoToBallState()
            elif my_car.boost < 33:
                self.state = GrabBoostState()
            elif my_car.boost > 33:
                self.state = GoToBallState()
            else:
                self.state = EmptyState()

        render_text = []
        render_text.append(self.state.__class__.__name__)
        render_text.append(self.state.mechanic.__class__.__name__)
        render_text.append(str(distance_2d(ball_location, car_location)))
        debug_text = "\n".join(render_text)
        draw_debug(self.renderer, car_location, debug_text)

        return self.controller_state


def draw_debug(renderer, location, action_display):
    renderer.begin_rendering()
    renderer.draw_string_3d(location, 2, 2,
                            action_display, renderer.white())
    renderer.end_rendering()
