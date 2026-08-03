"""Pure-pursuit path follower for a mecanum base (framework-independent).

Ported from the validated Webots `navigation.py`, but decoupled from any
sensor: you feed it the current pose (x, y, yaw) each tick and it returns a
body-frame twist (vx, vy, wz). The ROS 2 `navigation_node` calls `step()`
with the pose from /odom and publishes the twist on /cmd_vel.
"""

from __future__ import annotations

import math


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _clip(v, lo, hi):
    return max(lo, min(hi, v))


class PurePursuit:
    def __init__(self, position_tolerance=0.15, lookahead=0.4, cruise_speed=0.5,
                 brake_distance=0.6, min_speed=0.12, k_angular=2.0):
        self.position_tolerance = position_tolerance
        self.lookahead = lookahead
        self.cruise_speed = cruise_speed
        self.brake_distance = brake_distance
        self.min_speed = min_speed
        self.k_angular = k_angular
        self.waypoints: list[tuple[float, float]] = []
        self._i = 0
        self._t = 0.0

    def set_path(self, waypoints) -> None:
        self.waypoints = list(waypoints)
        self._i = 0
        self._t = 0.0

    def is_finished(self) -> bool:
        if not self.waypoints:
            return True
        return self._i >= len(self.waypoints) - 1 and self._t >= 1.0

    def step(self, x, y, yaw):
        """Return (status, vx, vy, wz). status in {'running','success','idle'}."""
        if not self.waypoints:
            return "idle", 0.0, 0.0, 0.0
        final = self.waypoints[-1]
        dist_final = math.hypot(final[0] - x, final[1] - y)
        if dist_final < self.position_tolerance:
            self._i = len(self.waypoints) - 1
            self._t = 1.0
            return "success", 0.0, 0.0, 0.0

        self._advance_cursor((x, y))
        tx, ty = self._lookahead_target()
        dx, dy = tx - x, ty - y
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return "success", 0.0, 0.0, 0.0

        speed = self.cruise_speed
        if dist_final < self.brake_distance:
            speed = max(self.min_speed, self.cruise_speed * dist_final / self.brake_distance)

        # HEADING FIRST, THEN DRIVE -- do not crab along the path.
        #
        # The first version resolved the world-frame velocity into the body
        # frame and let the base translate holonomically toward the target
        # WHILE also turning toward it. That is legal for a mecanum base and
        # it looks clever, but it means the body points one way and travels
        # another: the robot crabs diagonally down a corridor it should be
        # driving straight along. Two things follow, and both were measured.
        #
        # It makes the robot wider. A 0.58 x 0.38 m body crossing an aisle at
        # 45 deg sweeps 0.68 m of it instead of 0.38 m.
        #
        # And it aims the protective stop at the wrong thing. safety_node
        # tests the corridor swept ALONG THE COMMANDED DIRECTION; when that
        # direction is oblique, the test rectangle points into the gutter
        # beside the robot rather than down the lane ahead of it, so the guard
        # brakes for an obstacle the robot was never going to reach. The run
        # of 2026-08-03 17:43 spent 46% of its time blocked inside 1.05 m
        # margin lanes it could have driven straight down.
        #
        # So: turn to face the target, and translate along the body x axis
        # scaled by how well we are pointed. cos(err) falls to zero at 90 deg,
        # which turns the controller into a pure rotation until the heading
        # catches up, and is negative behind the robot, where translating
        # would only take it further away. The holonomic freedom is not lost;
        # it is reserved for the two places that genuinely need it -- the
        # sideways escape (navigation_node) and the visual alignment creep
        # (mission_node), both of which command the base directly.
        heading_error = _wrap(math.atan2(dy, dx) - yaw)
        wz = _clip(self.k_angular * heading_error, -1.5, 1.5)
        align = math.cos(heading_error)
        vx = speed * align if align > 0.0 else 0.0
        return "running", vx, 0.0, wz

    def _advance_cursor(self, pos):
        while self._i < len(self.waypoints) - 1:
            a = self.waypoints[self._i]
            b = self.waypoints[self._i + 1]
            sdx, sdy = b[0] - a[0], b[1] - a[1]
            seg2 = sdx * sdx + sdy * sdy
            if seg2 < 1e-9:
                self._i += 1
                self._t = 0.0
                continue
            t = ((pos[0] - a[0]) * sdx + (pos[1] - a[1]) * sdy) / seg2
            if t >= 1.0:
                self._i += 1
                self._t = 0.0
                continue
            self._t = max(self._t, max(0.0, t))
            return
        self._t = 1.0

    def _lookahead_target(self):
        remaining = self.lookahead
        i, t = self._i, self._t
        while i < len(self.waypoints) - 1:
            a = self.waypoints[i]
            b = self.waypoints[i + 1]
            sdx, sdy = b[0] - a[0], b[1] - a[1]
            seg = math.hypot(sdx, sdy)
            if seg < 1e-9:
                i += 1
                t = 0.0
                continue
            on_seg = (1.0 - t) * seg
            if remaining <= on_seg:
                frac = t + remaining / seg
                return a[0] + frac * sdx, a[1] + frac * sdy
            remaining -= on_seg
            i += 1
            t = 0.0
        return self.waypoints[-1]
