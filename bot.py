from copy import deepcopy
from typing import Tuple
import itertools
import os
import math
import collections

from pygame import Vector2
import pygame

from ...bot import Bot
from ...linear_math import Transform

# PHYSICS
# - steering is 60% effective (rest is drifting)
# - steering speed = 180 deg/sec
# - speed measured in pixels/sec
# - max throttle ac-/deceleration = 100 pixels/sec2
# - sideways (drifting) deceleration = 200 pixels/sec2

DEBUG = True
DRAW_FLAME = True

# TUNING PARAMETERS
EFFECTIVE_DECELERATION = -155  # pixels/sec2
STEER_DISTANCE_LIMIT = 42  # pixels
STEER_ANGLE_LIMIT = 42  # degrees
SPEED_LIMIT_FACTOR = 105  # pixels/sec ratio
CORNER_CUT_FACTOR = 50  # percent
CORNER_CUT_DISTANCE = 0  # pixels
STEERING_FACTOR = 60  # percent

# TRACK1: 90.817 (WITH SPINOUT)
# TRACK2: 78.100

# TIMES TO BEAT
# TRACK1: 97.150 - CaravanRacer
# TRACK2: 93.450 - Schummi


def lerp(min, max, factor):
    return (1 - factor) * min + factor * max


def inv_lerp(min, max, val):
    return (val - min) / (max - min)


def remap(min1, max1, min2, max2, val1):
    return lerp(min2, max2, inv_lerp(min1, max1, val1))


def sign(angle):
    return math.copysign(1, angle)


def normalizeAngle(angle):
    if angle > 180: return angle - 360
    if angle < -180: return 360 + angle
    return angle


def sliding_window(iterable, n):
    "Collect data into overlapping fixed-length chunks or blocks."
    # sliding_window('ABCDEFG', 4) → ABCD BCDE CDEF DEFG
    iterator = iter(iterable)
    window = collections.deque(itertools.islice(iterator, n - 1), maxlen=n)
    for x in iterator:
        window.append(x)
        yield tuple(window)


class Track:

    class Point:

        def __init__(self, parent, index, p):
            self._parent = parent
            self.index = index
            self.point = p

        def prv(self):
            return self._parent.get_item_wrap(self.index - 1)

        def nxt(self):
            return self._parent.get_item_wrap(self.index + 1)

        def vector_to_nxt(self):
            return self.nxt().point - self.point

        def dist_to_nxt(self):
            return self.vector_to_nxt().length()

        def dist_closest(self):
            return min(self.prv().dist_to_nxt(), self.dist_to_nxt())

        def angle(self):
            return normalizeAngle(self.prv().vector_to_nxt().angle_to(
                self.vector_to_nxt()))

        def is_apex(self):
            a_prv = self.prv().angle()
            a_nxt = self.nxt().angle()
            a = self.angle()
            return all(
                abs(a) > abs(x) for x in [a_prv, a_nxt] if sign(a) == sign(x))

        def next_apex(self):
            for i in range(self.index + 1, self.index + self._parent.length()):
                p = self._parent.get_item_wrap(i)
                if p.is_apex(): return p

        def prev_apex(self):
            for i in range(self.index - 1, self.index - self._parent.length(),
                           -1):
                p = self._parent.get_item_wrap(i)
                if p.is_apex(): return p

        def dist_to(self, index):
            result = 0
            p = self
            while p.index != index:
                result += p.dist_to_nxt()
                p = p.nxt()
            return result

    def __init__(self, track_lines):
        self._original_track_lines = track_lines

        self._points = [
            Track.Point(self, i, p)
            for i, p in enumerate(self._original_track_lines)
        ]

    def __getitem__(self, key):
        return self._points[key]

    def _normalize_key(self, key):
        return key % len(self._points)

    def get_item_wrap(self, key):
        return self.__getitem__(self._normalize_key(key))

    def length(self):
        return len(self._points)


class Gonzales(Bot):

    def __init__(self,
                 track,
                 *,
                 effective_deceleration=EFFECTIVE_DECELERATION,
                 steer_distance_limit=STEER_DISTANCE_LIMIT,
                 steer_angle_limit=STEER_ANGLE_LIMIT,
                 speed_limit_factor=SPEED_LIMIT_FACTOR,
                 corner_cut_factor=CORNER_CUT_FACTOR,
                 corner_cut_distance=CORNER_CUT_DISTANCE,
                 steering_factor=STEERING_FACTOR):
        super().__init__(track)

        self._effective_deceleration = effective_deceleration
        self._steer_distance_limit = steer_distance_limit
        self._steer_angle_limit = steer_angle_limit
        self._speed_limit_factor = speed_limit_factor
        self._corner_cut_factor = corner_cut_factor
        self._corner_cut_distance = corner_cut_distance
        self._steering_factor = steering_factor

        self._original_track = Track(self.track.lines)
        self._track = self._optimizeTrack()

        self._speed_limits = [
            self._speed_limit_factor * p.dist_closest() / abs(p.angle())
            for p in self._track
        ]

        if DRAW_FLAME:
            self._flame = pygame.image.load(
                os.path.dirname(__file__) + '/flame.png')

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

    def _optimizeTrack(self):
        ccf = []
        for p in self._original_track:
            should_cut = False
            if p.is_apex():
                should_cut = True
            else:
                next_apex = p.next_apex()
                prev_apex = p.prev_apex()

                # should_cut = (sign(next_apex.angle()) == sign(
                #     prev_apex.angle())) and (p.dist_to(next_apex.index) < 200
                #                              or
                #                              prev_apex.dist_to(p.index) < 200)
                should_cut = (
                    p.dist_to(next_apex.index) < self._corner_cut_distance
                    or prev_apex.dist_to(p.index) < self._corner_cut_distance)

            ccf.append(self._corner_cut_factor if should_cut else 0)
        ccf = [
            c if c > 0 else max(p, n) / 2
            for p, c, n in sliding_window(ccf[-1:] + ccf + ccf[:1], 3)
        ]

        result = [
            p.prv().vector_to_nxt().project(p.prv().vector_to_nxt() +
                                            p.vector_to_nxt()) -
            p.prv().vector_to_nxt() for p in self._original_track
        ]
        result = [
            c + (v.normalize() * min(self.track.track_width *
                                     (f / 100), v.length()))
            for c, v, f in zip(self.track.lines, result, ccf)
        ]

        return Track(result)

    def _goFast(self, wp, pos, velocity):
        target = self._track.get_item_wrap(wp)

        throttle = 1

        v = velocity.length()
        mt = -v / self._effective_deceleration
        md = 0.5 * self._effective_deceleration * mt * mt + v * mt

        d = (pos.p - target.point).length()
        self._last_brake_wp = None  # debug
        while md > d:
            tv = self._speed_limits[target.index]
            ts = (tv - v) / self._effective_deceleration
            td = 0.5 * self._effective_deceleration * ts * ts + v * ts
            if td > d:
                throttle = -1
                self._last_brake_wp = target.index  # debug
                break
            target = target.nxt()
            d += target.dist_to_nxt()

        return throttle

    def _inTheRightDirection(self, wp, pos, velocity):
        target = self._track.get_item_wrap(wp)

        posinverse = deepcopy(pos).inverse()
        angle = (posinverse * target.point).as_polar()[1]

        d = (pos.p - target.point).length()
        if (d < self._steer_distance_limit) and (abs(angle) <
                                                 self._steer_angle_limit):
            target = target.nxt()
            angle = (posinverse * target.point).as_polar()[1]

        if DEBUG: self._last_steer_wp = wp

        return angle * (self._steering_factor / 100)

    def draw(self, map_scaled, zoom):

        if DRAW_FLAME:
            flame_pos = self._last_position
            flame_zoom = 0.1 * zoom
            flame_angle = flame_pos.M.angle
            flame_image = pygame.transform.rotozoom(
                self._flame, -math.degrees(flame_angle) - 45, flame_zoom)
            flame_rect = flame_image.get_rect(
                center=(flame_pos.p - flame_pos.M * Vector2(40, 0)) * zoom)
            map_scaled.blit(flame_image, flame_rect)

        if not DEBUG: return

        # ANGLES
        # for c, a in zip(self.track.lines, self._angles):
        #     text = self._font.render(f'{a:.2f}', True,
        #                              self._red if a > 0 else self._black)
        #     map_scaled.blit(text, c * zoom)

        # VELOCITY VECTOR
        # pygame.draw.line(map_scaled, self._black,
        #                  self._last_coordinates * zoom,
        #                  (self._last_coordinates + self._last_velocity) * zoom,
        #                  2)

        # SPEED LIMITS
        # for i in range(len(self.track.lines)):
        #     p = self.track.lines[i] * zoom
        #     m = self._speed_limits[i]
        #     text = self._font.render(
        #         f'{m:.2f}', True, self._red
        #         if self._original_track[i].is_apex() else self._black)
        #     map_scaled.blit(text, p)

        # STEER WP
        pygame.draw.line(
            map_scaled, self._green, self._last_coordinates * zoom,
            self._track.get_item_wrap(self._last_steer_wp).point * zoom, 2)

        # BRAKE WP
        if self._last_brake_wp is not None:
            pygame.draw.line(
                map_scaled, self._red, self._last_coordinates * zoom,
                self._track.get_item_wrap(self._last_brake_wp).point * zoom, 2)

        for c, p in zip(self.track.lines, self._track):
            pygame.draw.line(map_scaled, self._red, c * zoom, p.point * zoom)

        color = self._green if self._last_throttle > 0 else self._red

        text = self._font.render(f'{self._last_speed:.2f}', True, color)
        map_scaled.blit(text, (self._last_coordinates * zoom) -
                        pygame.Vector2(25, 25))

    def compute_commands(self, next_waypoint: int, position: Transform,
                         velocity: Vector2) -> Tuple:

        throttle = self._goFast(next_waypoint, position, velocity)
        steering = self._inTheRightDirection(next_waypoint, position, velocity)

        if DRAW_FLAME:
            self._last_position = position

        if DEBUG:
            self._last_wp = next_waypoint
            self._last_coordinates = position.p
            self._last_velocity = velocity
            self._last_speed = velocity.length()
            self._last_throttle = throttle

        return throttle, steering
