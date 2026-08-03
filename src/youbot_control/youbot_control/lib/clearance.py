"""clearance -- footprint-relative lidar clearance, shared by the guard and
the follower.

Why this file exists. safety_node measured the protective stop from the LIDAR
ORIGIN, which sits at the centre of the base. The base is 0.58 x 0.38 m, so its
front bumper is already 0.29 m ahead of that origin -- and stop_distance was
0.28 m. The robot therefore braked when the wall was 0.28 m from its centre,
i.e. 1 cm INSIDE its own nose. That is exactly the picture from the field: the
chassis buried a quarter of a metre into the glass, the brake holding
translation, the stuck detector firing over and over at the same spot for nine
minutes. The base has no collision geometry (it is driven kinematically), so
nothing in the physics stops it either -- the only thing between the robot and
the wall is this number, and it was measured from the wrong place.

So: every distance here is measured from the FOOTPRINT EDGE along the direction
of travel, not from the sensor. `stop_distance` then means what it reads --
clearance in front of the bumper.

Both safety_node and navigation_node use these functions, so the guard that
brakes and the recovery that chooses where to escape agree about what "blocked"
means. They disagreed before, which is how the escape manoeuvre kept reversing
into a direction the guard immediately cancelled.
"""

from __future__ import annotations

import math


def footprint_reach(direction: float, half_length: float,
                    half_width: float) -> float:
    """Distance from the base centre to the edge of the footprint rectangle
    along `direction` (body frame, 0 = forward).

    0.29 m straight ahead, 0.19 m straight sideways, and the rectangle
    boundary in between -- not a disc: a 0.347 m circumscribed circle would
    brake for a wall the robot is merely driving alongside.
    """
    c, s = abs(math.cos(direction)), abs(math.sin(direction))
    a = half_length / c if c > 1e-9 else float("inf")
    b = half_width / s if s > 1e-9 else float("inf")
    return min(a, b)


def corridor_clearance(pts, direction: float, half_width: float,
                       half_length: float, body_half_width: float) -> float:
    """Free distance IN FRONT OF THE FOOTPRINT along `direction`.

    `pts` are body-frame (x, y) lidar returns. A return counts only if it
    falls inside the rectangle the base sweeps going that way (half_width to
    each side); the result is its distance along the direction of travel,
    minus how far the body already reaches that way. inf when clear, 0.0 when
    something is level with the bumper or behind it.
    """
    ux, uy = math.cos(direction), math.sin(direction)
    best = float("inf")
    for px, py in pts:
        along = px * ux + py * uy
        if along <= 0.0 or along >= best:
            continue
        if abs(-px * uy + py * ux) <= half_width:
            best = along
    if best == float("inf"):
        return best
    return max(0.0, best - footprint_reach(direction, half_length,
                                           body_half_width))


def best_escape(pts, half_width: float, half_length: float,
                body_half_width: float, avoid: float | None = None,
                n: int = 12):
    """Pick the body-frame direction with the most room in front of it.

    Returns (direction, clearance). `avoid` is the direction we are already
    trying and failing to go: headings within 60 deg of it are skipped, so the
    escape cannot be "keep pushing into the same wall". Sampling all around
    (not just backwards) is the point -- at the end of an aisle the way out is
    usually sideways, and a fixed reverse-and-turn manoeuvre spends its two
    seconds backing into the corner it came from.
    """
    best_dir, best_gap = None, -1.0
    for i in range(n):
        d = -math.pi + 2.0 * math.pi * i / n
        if avoid is not None:
            delta = abs(math.atan2(math.sin(d - avoid), math.cos(d - avoid)))
            if delta < math.radians(60.0):
                continue
        gap = corridor_clearance(pts, d, half_width, half_length,
                                 body_half_width)
        if gap > best_gap:
            best_dir, best_gap = d, gap
    return best_dir, best_gap
