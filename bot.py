from copy import deepcopy
from typing import Tuple
import itertools

from pygame import Vector2
import pygame

from ...bot import Bot
from ...linear_math import Transform

# IDEAS
# - more granular throttle
# - better estimation of max speed
# - drifting seems suboptimal. perhaps cutting corners is better.

# PHYSICS
# - steering is 60% effective (rest is drifting)
# - steering speed = 180 deg/sec
# - speed measured in pixels/sec
# - max throttle ac-/deceleration = 100 pixels/sec2
# - sideways (drifting) deceleration = 200 pixels/sec2

DEBUG = False

# TUNING PARAMETERS
EFFECTIVE_DECELERATION = -114
STEER_DISTANCE_LIMIT = 64
STEER_ANGLE_LIMIT = 28
SPEED_LIMIT_FACTOR = 115


def normalizeAngle(angle):
    angle = abs(angle)
    if angle > 180: angle = 360 - angle
    return angle


class Gonzales(Bot):

    def __init__(self,
                 track,
                 *,
                 effective_deceleration=EFFECTIVE_DECELERATION,
                 steer_distance_limit=STEER_DISTANCE_LIMIT,
                 steer_angle_limit=STEER_ANGLE_LIMIT,
                 speed_limit_factor=SPEED_LIMIT_FACTOR):
        super().__init__(track)

        self._effective_deceleration = effective_deceleration
        self._steer_distance_limit = steer_distance_limit
        self._steer_angle_limit = steer_angle_limit
        self._speed_limit_factor = speed_limit_factor

        self._track_len = len(self.track.lines)

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
        self._speed_limits = [
            self._speed_limit_factor * d / a
            for a, d in zip(angles, distances)
        ]
        self._distances = [v.length() for v in vectors]

        if DEBUG:
            self._font = pygame.font.SysFont(None, 24)
            self._black = pygame.Color(0, 0, 0, 50)
            self._green = pygame.Color(0, 255, 0, 50)
            self._red = pygame.Color(255, 0, 0, 50)

    @property
    def name(self):
        return "Gonzales"

    @property
    def contributor(self):
        return "Lewie"

    def _incWP(self, wp):
        return (wp + 1) % self._track_len

    def _goFast(self, wp, pos, velocity):
        target = self.track.lines[wp]

        throttle = 1

        v = velocity.length()
        mt = -v / self._effective_deceleration
        md = 0.5 * self._effective_deceleration * mt * mt + v * mt

        d = (pos.p - target).length()
        while md > d:
            tv = self._speed_limits[wp]
            ts = (tv - v) / self._effective_deceleration
            td = 0.5 * self._effective_deceleration * ts * ts + v * ts
            if td > d:
                throttle = -1
                break
            wp = self._incWP(wp)
            d += self._distances[wp]

        return throttle

    def _thatWay(self, wp, pos, velocity):
        target = self.track.lines[wp]

        posinverse = deepcopy(pos).inverse()
        angle = (posinverse * target).as_polar()[1]

        d = (pos.p - target).length()
        if (d < self._steer_distance_limit) and (abs(angle) <
                                                 self._steer_angle_limit):
            target = posinverse * self.track.lines[self._incWP(wp)]
            angle = target.as_polar()[1]

        return angle / 3

    def draw(self, map_scaled, zoom):
        if not DEBUG: return

        # VELOCITY VECTOR
        # pygame.draw.line(map_scaled, self._black, self.last_coordinates * zoom,
        #                  (self.last_coordinates + self.last_velocity) * zoom,
        #                  2)

        # SPEED LIMITS
        # for i in range(len(self.track.lines)):
        #     p = self.track.lines[i] * zoom
        #     m = self._speed_limits[i]
        #     text = self._font.render(f'{m:.2f}', True, self._black)
        #     map_scaled.blit(text, p)

        color = self._green if self.last_throttle > 0 else self._red

        text = self._font.render(f'{self.last_speed:.2f}', True, color)
        map_scaled.blit(text, (self.last_coordinates * zoom) -
                        pygame.Vector2(25, 25))

    def compute_commands(self, next_waypoint: int, position: Transform,
                         velocity: Vector2) -> Tuple:

        throttle = self._goFast(next_waypoint, position, velocity)
        steering = self._thatWay(next_waypoint, position, velocity)

        if DEBUG:
            self.last_coordinates = position.p
            self.last_velocity = velocity
            self.last_speed = velocity.length()
            self.last_throttle = throttle

        return throttle, steering
