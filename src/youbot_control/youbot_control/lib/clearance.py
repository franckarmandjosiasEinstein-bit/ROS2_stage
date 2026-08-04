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


def swept_half_width(direction: float, half_length: float,
                     half_width: float) -> float:
    """Half-width of the band the BASE ACTUALLY SWEEPS going `direction`.

    This used to be a constant, 0.24 m, and a constant cannot be right: the
    base is a 0.58 x 0.38 m rectangle that does not turn to face where it is
    going, so the width of its shadow depends entirely on the direction of
    travel. Projecting the rectangle onto the axis normal to the motion gives

        a*|sin d| + b*|cos d|      a = half length, b = half width

    which is 0.19 m straight ahead, 0.29 m straight sideways, and 0.34 m on
    the diagonal. The single value 0.24 was therefore wrong in both
    directions at once, and each error had its own symptom:

      TOO WIDE going forward (0.24 vs 0.19). In a 0.80 m aisle the free space
      beside a centred base is 0.21 m, so 5 cm of phantom band is a quarter
      of the whole margin. Measured against the real gutter geometry: at a
      lateral offset of 0.18 m -- body edge still 3 cm clear, driving exactly
      parallel to the wall, closing on nothing -- the constant band reports
      zero clearance and the guard holds translation. That is the
      "Translation held" beside the gutters, and it is a brake firing at a
      wall the robot is merely driving alongside, which is the exact failure
      the 0.24 comment claimed to prevent.

      TOO NARROW going sideways (0.24 vs 0.29). Visual alignment strafes.
      During a strafe the base sweeps 0.29 m to each side and the guard was
      only watching 0.24, so 5 cm of the moving footprint was outside the
      test. That is not a nuisance, it is a hole: the gutter the robot
      entered during alignment was in that gap.

    So the band is computed, and `margin` is what is added on top of the true
    geometry -- an actual safety margin instead of a number chosen to be a bad
    compromise between two directions.

    Kept as a named function because it is the number to quote in a report and
    in a log line, but the clearance itself no longer uses it: see
    `contact_distance`, which is exact where a band-plus-constant-reach is
    only an approximation.
    """
    return (half_length * abs(math.sin(direction))
            + half_width * abs(math.cos(direction)))


def contact_distance(px: float, py: float, direction: float,
                     half_length: float, half_width: float,
                     margin: float = 0.0) -> float | None:
    """How far the base can travel along `direction` before it touches (px, py).

    None when it never touches. This is a ray-box intersection run backwards:
    the body is the rectangle |x| <= a+margin, |y| <= b+margin, and we want the
    smallest t with the point inside the body translated by t*u -- that is, the
    smallest t such that |px - t*ux| <= a and |py - t*uy| <= b.

    WHY THE EXACT VERSION IS NEEDED. The previous code answered this in two
    approximate halves: a lateral band test to decide whether the point counts
    at all, then a single `footprint_reach(direction)` subtracted from its
    distance along the path. That reach is measured along the CENTRELINE, but
    the part of the base that arrives first is a corner. Travelling at 45 deg
    the centreline reach is 0.269 m while the leading corner is already 0.339 m
    out, so the guard credited itself 7 cm of room it did not have -- in the
    direction the robot crosses an 0.80 m aisle, where 7 cm is a third of the
    margin. Nothing about a band and a constant can fix that, because the reach
    genuinely varies with WHERE in the band the point sits: 0.339 m for a point
    near the leading corner, much less at the far edge of the band.

    The slab form below gives the true value for every point and every
    direction, so the band test and the constant reach both disappear into it.
    """
    ux, uy = math.cos(direction), math.sin(direction)
    a, b = half_length + margin, half_width + margin
    lo, hi = -math.inf, math.inf
    for p, u, h in ((px, ux, a), (py, uy, b)):
        if abs(u) < 1e-9:
            # No motion on this axis: the point must already be within the
            # body's extent on it, or the base never sweeps over it at all.
            if abs(p) > h:
                return None
            continue
        t1, t2 = (p - h) / u, (p + h) / u
        lo, hi = max(lo, min(t1, t2)), min(hi, max(t1, t2))
    if lo > hi or hi <= 0.0:
        return None            # never touches, or only behind us
    return max(0.0, lo)


def corridor_clearance(pts, direction: float, margin: float,
                       half_length: float, body_half_width: float) -> float:
    """Free distance IN FRONT OF THE FOOTPRINT along `direction`.

    `pts` are body-frame (x, y) lidar returns. For each one, how far the base
    can travel that way before touching it (`contact_distance`); the answer is
    the smallest. inf when nothing is ever touched, 0.0 when something is
    already level with the footprint.
    """
    best = float("inf")
    for px, py in pts:
        d = contact_distance(px, py, direction, half_length, body_half_width,
                             margin)
        if d is not None and d < best:
            best = d
    return best


def blocking_point(pts, direction: float, margin: float,
                   half_length: float, body_half_width: float):
    """The return that is actually stopping us, for diagnostics.

    Same test as `corridor_clearance`, but it hands back the offending
    point instead of only the distance. Written after two rounds of
    guessing at logs that said "Obstacle within 0.12 m" and nothing
    else: in a 1.05 m lane that message is a puzzle, and the answer is
    one number away. A protective stop that cannot say WHAT it stopped
    for cannot be debugged from a log.

    Returns (px, py, clearance) in the body frame, or None when clear.
    """
    best, hit = float("inf"), None
    for px, py in pts:
        d = contact_distance(px, py, direction, half_length, body_half_width,
                             margin)
        if d is not None and d < best:
            best, hit = d, (px, py)
    if hit is None:
        return None
    return hit[0], hit[1], best


def best_escape(pts, margin: float, half_length: float,
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
        gap = corridor_clearance(pts, d, margin, half_length,
                                 body_half_width)
        if gap > best_gap:
            best_dir, best_gap = d, gap
    return best_dir, best_gap
