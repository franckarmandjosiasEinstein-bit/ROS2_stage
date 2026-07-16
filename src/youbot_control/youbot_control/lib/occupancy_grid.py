"""Occupancy-grid mapping (framework-independent).

Ported from the validated Webots `mapping.py`: log-odds fusion of 2D lidar
scans with Bresenham ray-casting and obstacle inflation. No ROS/Webots
imports -- the ROS 2 `mapping_node` wraps this and converts the grid to a
`nav_msgs/OccupancyGrid`.

World frame: XY plane, origin at arena centre, +X east, +Y north.
Map frame:   row = Y axis, col = X axis, origin top-left.
"""

from __future__ import annotations

import math

import numpy as np

L_OCC = 0.85          # log-odds added where a beam ends (hit)
L_FREE = -0.40        # log-odds added along the free path
L_CLAMP = 5.0         # keep belief bounded so the map stays responsive
L_OCC_THRESHOLD = 0.5  # above this => occupied in the binary grid


def world_to_map(x: float, y: float, resolution: float, size: float) -> tuple[int, int]:
    col = int((x + size / 2.0) / resolution)
    row = int((size / 2.0 - y) / resolution)
    return col, row


def map_to_world(col: int, row: int, resolution: float, size: float) -> tuple[float, float]:
    x = (col + 0.5) * resolution - size / 2.0
    y = size / 2.0 - (row + 0.5) * resolution
    return x, y


class OccupancyGrid:
    """Log-odds occupancy grid built incrementally from lidar scans."""

    def __init__(self, resolution: float = 0.10, arena_size: float = 10.0,
                 inflation: float = 0.32) -> None:
        self.resolution = resolution
        self.arena_size = arena_size
        self.inflation = inflation
        cells = int(round(arena_size / resolution))
        self.log_odds = np.zeros((cells, cells), dtype=np.float32)
        self.grid = np.zeros((cells, cells), dtype=np.uint8)  # 1 = obstacle (inflated)

    @property
    def shape(self) -> tuple[int, int]:
        return self.grid.shape

    def integrate_scan(self, ranges, robot_x, robot_y, robot_yaw,
                       angle_min, angle_increment,
                       max_range, min_range: float = 0.25) -> None:
        """Fuse one 2D LaserScan into the belief.

        Beam i points at `angle_min + i*angle_increment` in the robot frame
        (the ROS sensor_msgs/LaserScan convention), which works whatever the
        scan direction -- Webots' lidar sweeps clockwise (angle_min = +pi,
        negative increment), so deriving a single `fov` and assuming a fixed
        sweep direction would mirror the map.
        """
        n = len(ranges)
        if n == 0:
            return
        rows, cols = self.log_odds.shape
        r0c, r0r = world_to_map(robot_x, robot_y, self.resolution, self.arena_size)
        if not (0 <= r0c < cols and 0 <= r0r < rows):
            return
        for i, dist in enumerate(ranges):
            angle = robot_yaw + angle_min + i * angle_increment
            hit = math.isfinite(dist) and dist < max_range
            reach = dist if hit else max_range
            if reach < min_range:
                continue
            ex = robot_x + reach * math.cos(angle)
            ey = robot_y + reach * math.sin(angle)
            ec, er = world_to_map(ex, ey, self.resolution, self.arena_size)
            for cc, cr in self._bresenham(r0c, r0r, ec, er):
                if cc == ec and cr == er:
                    break
                if 0 <= cc < cols and 0 <= cr < rows:
                    self.log_odds[cr, cc] += L_FREE
            if hit and 0 <= ec < cols and 0 <= er < rows:
                self.log_odds[er, ec] += L_OCC
        np.clip(self.log_odds, -L_CLAMP, L_CLAMP, out=self.log_odds)

    def update_binary(self) -> None:
        """Rebuild the inflated binary planning grid from the belief."""
        occupied = self.log_odds > L_OCC_THRESHOLD
        self.grid = self._inflate(occupied).astype(np.uint8)

    def is_free(self, col: int, row: int) -> bool:
        rows, cols = self.grid.shape
        if not (0 <= col < cols and 0 <= row < rows):
            return False
        return self.grid[row, col] == 0

    def _inflate(self, occupied: np.ndarray) -> np.ndarray:
        radius = int(round(self.inflation / self.resolution))
        if radius <= 0:
            return occupied.copy()
        out = occupied.copy()
        rows, cols = occupied.shape
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                if dr * dr + dc * dc > radius * radius:
                    continue
                out[max(0, dr):rows - max(0, -dr), max(0, dc):cols - max(0, -dc)] |= \
                    occupied[max(0, -dr):rows - max(0, dr), max(0, -dc):cols - max(0, dc)]
        return out

    @staticmethod
    def _bresenham(c0: int, r0: int, c1: int, r1: int):
        dc = abs(c1 - c0)
        dr = abs(r1 - r0)
        sc = 1 if c0 < c1 else -1
        sr = 1 if r0 < r1 else -1
        err = dc - dr
        c, r = c0, r0
        while True:
            yield c, r
            if c == c1 and r == r1:
                return
            e2 = 2 * err
            if e2 > -dr:
                err -= dr
                c += sc
            if e2 < dc:
                err += dc
                r += sr
