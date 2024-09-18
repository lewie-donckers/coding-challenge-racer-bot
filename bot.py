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

        next_next_waypoint = next_waypoint + 1
        if next_next_waypoint >= len(self.track.lines): next_next_waypoint = 0
        next_target = self.track.lines[next_next_waypoint]
        next_target = position.inverse() * next_target
        next_angle = next_target.as_polar()[1]

        go_back = False
        abs_angle = abs(angle)
        if abs_angle > 90:
            go_back = True
            abs_angle = abs(abs_angle - 180)

        # calculate the throttle
        target_velocity = max(100, 300 - (abs_angle * 3))
        if velocity.length() < target_velocity:
            throttle = 1
        else:
            throttle = -1

        steering = 0
        if abs_angle > 1:
            if angle > 0:
                steering = 1
            else:
                steering = -1

        if go_back:
            steering = steering * -1
            throttle = throttle * -1

        return throttle, steering
