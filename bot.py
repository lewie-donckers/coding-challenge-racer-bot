from typing import Tuple
import itertools

from pygame import Vector2
import pygame

from ...bot import Bot
from ...linear_math import Transform

# IDEAS
# - more granular throttle and steering
# - better estimation of max speed

# BUGS
# - if going backwards from target (through skidding) but abs speed is lower than target, it will accelerate backwards

# PHYSICS
# - steering is 60% effective (rest is drifting)
# - steering speed = 180 deg/sec
# - speed measured in pixels/sec
# - max throttle ac-/deceleration = 100 pixels/sec2
# - sideways (drifting) deceleration = 200 pixels/sec2

DEBUG = False

EFFECTIVE_DECELERATION = -125


def normalizeAngle(angle):
    angle = abs(angle)
    if angle > 180: angle = 360 - angle
    return angle


class Gonzales(Bot):

    def __init__(self, track):
        super().__init__(track)

        coordinates = [self.track.lines[-1]
                       ] + self.track.lines + [self.track.lines[0]]
        vectors = [c1 - c0 for c0, c1 in itertools.pairwise(coordinates)]
        angles = [
            normalizeAngle(v0.angle_to(v1))
            for v0, v1 in itertools.pairwise(vectors)
        ]
        distances = [
            min(v0.length(), v1.length())
            for v0, v1 in itertools.pairwise(vectors)
        ]
        self.max_speeds = [100 * d / a for a, d in zip(angles, distances)]
        self.distances = [v.length() for v in vectors]

        if DEBUG:
            self.font = pygame.font.SysFont(None, 24)
            self.black = pygame.Color(0, 0, 0, 50)
            self.green = pygame.Color(0, 255, 0, 50)
            self.red = pygame.Color(255, 0, 0, 50)

    @property
    def name(self):
        return "Gonzales"

    @property
    def contributor(self):
        return "Lewie"

    def draw(self, map_scaled, zoom):
        if not DEBUG: pass

        for i in range(len(self.track.lines)):
            p = self.track.lines[i] * zoom
            m = self.max_speeds[i]
            text = self.font.render(f'{m:.2f}', True, self.black)
            map_scaled.blit(text, p)

        color = self.green if self.last_throttle > 0 else self.red

        text = self.font.render(f'{self.last_speed:.2f}', True, color)
        map_scaled.blit(text, (self.last_coordinates * zoom) -
                        pygame.Vector2(25, 25))

    def compute_commands(self, next_waypoint: int, position: Transform,
                         velocity: Vector2) -> Tuple:
        self.last_coordinates = position.p
        self.last_speed = velocity.length()

        target = self.track.lines[next_waypoint]

        # THROTTLE
        throttle = 1

        v = velocity.length()
        mt = -v / EFFECTIVE_DECELERATION
        md = 0.5 * EFFECTIVE_DECELERATION * mt * mt + v * mt

        d = (position.p - target).length()
        while md > d:
            tv = self.max_speeds[next_waypoint]
            ts = (tv - v) / EFFECTIVE_DECELERATION
            td = 0.5 * EFFECTIVE_DECELERATION * ts * ts + v * ts
            if td > d:
                throttle = -1
                break
            next_waypoint = (next_waypoint + 1) % len(self.track.lines)
            d += self.distances[next_waypoint]

        # STEERING
        target = position.inverse() * target
        angle = target.as_polar()[1]

        steering = 0
        if abs(angle) > 1:
            if angle > 0:
                steering = 1
            else:
                steering = -1

        self.last_throttle = throttle

        return throttle, steering
