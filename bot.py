from typing import Tuple

from pygame import Vector2

from ...bot import Bot
from ...linear_math import Transform


class Gonzales(Bot):

    @property
    def name(self):
        return "Gonzales"

    @property
    def contributor(self):
        return "Lewie"

    def compute_commands(self, next_waypoint: int, position: Transform,
                         velocity: Vector2) -> Tuple:
        target = self.track.lines[next_waypoint]
        # calculate the target in the frame of the robot
        target = position.inverse() * target
        # calculate the angle to the target
        angle = target.as_polar()[1]

        # calculate the throttle
        target_velocity = 200
        if velocity.length() < target_velocity:
            throttle = 1
        else:
            throttle = -1

        # calculate the steering
        if angle > 0:
            return throttle, 1
        else:
            return throttle, -1
