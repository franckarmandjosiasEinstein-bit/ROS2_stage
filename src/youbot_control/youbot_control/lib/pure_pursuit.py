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

        vx_w, vy_w = speed * dx / norm, speed * dy / norm
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        vx = cos_y * vx_w + sin_y * vy_w        # world -> body
        vy = -sin_y * vx_w + cos_y * vy_w
        wz = _clip(self.k_angular * _wrap(math.atan2(dy, dx) - yaw), -1.5, 1.5)
        return "running", vx, vy, wz

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
