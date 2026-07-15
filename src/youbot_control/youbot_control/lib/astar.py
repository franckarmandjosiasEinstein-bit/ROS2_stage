"""A* path planning on an occupancy grid (framework-independent).

Ported from the validated Webots `planning.py`: 8-connectivity, octile
heuristic, start/goal snap-to-free, collinear smoothing. Operates on any
object exposing `.grid` (numpy, 1 = obstacle), `.resolution`, `.arena_size`
and `world_to_map`/`map_to_world` from `occupancy_grid`.
"""

from __future__ import annotations

import heapq
import math

from .occupancy_grid import map_to_world, world_to_map

NEIGHBORS = (
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
)


def _heuristic(a, b):
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return math.sqrt(2) * min(dx, dy) + abs(dx - dy)


def plan(grid, resolution, arena_size, start_world, goal_world):
    """Return a list of world-frame (x, y) waypoints, or [] if unreachable."""
    start = world_to_map(*start_world, resolution=resolution, size=arena_size)
    goal = world_to_map(*goal_world, resolution=resolution, size=arena_size)
    rows, cols = grid.shape

    def in_bounds(c):
        return 0 <= c[0] < cols and 0 <= c[1] < rows

    def is_free(c):
        return grid[c[1], c[0]] == 0

    def nearest_free(cell):
        if in_bounds(cell) and is_free(cell):
            return cell
        for radius in range(1, max(rows, cols)):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if max(abs(dr), abs(dc)) != radius:
                        continue
                    cand = (cell[0] + dc, cell[1] + dr)
                    if in_bounds(cand) and is_free(cand):
                        return cand
        return None

    start = nearest_free(start)
    goal = nearest_free(goal)
    if start is None or goal is None:
        return []

    came_from = {}
    g_score = {start: 0.0}
    open_heap = [(_heuristic(start, goal), start)]
    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current == goal:
            return _reconstruct(came_from, current, resolution, arena_size)
        for dx, dy, cost in NEIGHBORS:
            nb = (current[0] + dx, current[1] + dy)
            if not in_bounds(nb) or not is_free(nb):
                continue
            tentative = g_score[current] + cost
            if tentative < g_score.get(nb, float("inf")):
                came_from[nb] = current
                g_score[nb] = tentative
                heapq.heappush(open_heap, (tentative + _heuristic(nb, goal), nb))
    return []


def _reconstruct(came_from, current, resolution, arena_size):
    cells = [current]
    while current in came_from:
        current = came_from[current]
        cells.append(current)
    cells.reverse()
    cells = _smooth(cells)
    return [map_to_world(c, r, resolution, arena_size) for c, r in cells]


def _smooth(cells):
    """Drop collinear intermediate cells to shorten the waypoint list."""
    if len(cells) < 3:
        return cells
    out = [cells[0]]
    for i in range(1, len(cells) - 1):
        prev = out[-1]
        d1 = (cells[i][0] - prev[0], cells[i][1] - prev[1])
        d2 = (cells[i + 1][0] - cells[i][0], cells[i + 1][1] - cells[i][1])
        if d1 != d2:
            out.append(cells[i])
    out.append(cells[-1])
    return out
