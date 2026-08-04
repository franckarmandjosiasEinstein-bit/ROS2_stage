"""aisles -- the free space, and how to get across it without a planner.

The greenhouse is known, static and rectangular. Its free space is four
aisles running along x, joined by a headland at each end:

    bands (in y)   between the walls (|y| < 2.45) and the three 0.40 m
                   gutters centred on y = -1.2, 0, +1.2
    headlands      past the ends of the 8 m gutters, |x| > 4.0

So a route is at most three legs: out to a headland, across to the target
band, in to the station. There is no planner because there is no question to
plan: A* on a grid would spend a millisecond rediscovering this paragraph,
and would then be free to discover something else on a day when the grid was
subtly wrong -- which is what happened in Phase B.

WHY THIS IS NOT IN THE DRIVER

Because then it could only be tested with Gazebo running, and it is the part
most worth testing: an error here does not look like an error, it looks like
a robot grinding along a gutter. Here, check_cloud.py sweeps all 2 256
station pairs and verifies that the robot's rectangle never touches a gutter
or a wall on any leg of any route. That takes a second and needs nothing
installed.

EVERY COORDINATE HERE IS THE SENSOR POINT

Not base_link. The robot's reference point is SENSOR_OFFSET_X ahead of the
base (see catalogue.py), so the body occupies
[x - SENSOR_OFFSET_X - BASE_HALF_LENGTH, x - SENSOR_OFFSET_X + BASE_HALF_LENGTH]
plus the boom out to x. Mixing the two frames is the single most likely way
to get this wrong, so only one of them appears.
"""

from __future__ import annotations

from agri.catalogue import (BASE_HALF_LENGTH, BASE_HALF_WIDTH,
                            GUTTER_HALF_WIDTH, ROW_Y, SENSOR_OFFSET_X, WALL_X,
                            WALL_Y)

#: Half-length of the gutters along x. They are 8.0 m long, centred on 0.
GUTTER_HALF_LENGTH = 4.0

#: The free bands in y, as (lo, hi) of the ROBOT CENTRE LINE. Derived, not
#: typed: the walls and the gutters decide them.
def _bands() -> tuple[tuple[float, float], ...]:
    edges = [-WALL_Y]
    for y in sorted(ROW_Y.values()):
        edges += [y - GUTTER_HALF_WIDTH, y + GUTTER_HALF_WIDTH]
    edges.append(WALL_Y)
    return tuple((edges[i], edges[i + 1]) for i in range(0, len(edges), 2))


BANDS = _bands()

#: The robot's total extent along x: boom tip to tail.
_SPAN_BACK = SENSOR_OFFSET_X + BASE_HALF_LENGTH
#: Margin left at each end of a headland when the robot is centred in it.
HEADLAND_MARGIN = round((WALL_X - GUTTER_HALF_LENGTH - _SPAN_BACK) / 2.0, 3)

#: SENSOR-POINT x of the west and east headlands.
#:
#: NOT symmetric, because the robot is not. The boom always points +x, so at
#: the WEST end the sensor point is the leading edge and must stop short of
#: the gutters, while at the EAST end it is the sensor point that goes deep
#: and the tail that must clear them. The footprint ends up centred in the
#: 0.95 m gap either way, with HEADLAND_MARGIN at each end.
#:
#: The first version of this file used one number for both and put it at the
#: gutter end. Going east, that left the chassis lying alongside the gutters
#: while the robot strafed across the greenhouse -- straight through them.
#: Nothing would have reported it: this robot has no collision geometry, so
#: driving through a gutter produces no contact, no warning and no complaint,
#: only a recording that cannot be shown to anyone. It was caught by
#: route_clearance() below, which is the entire reason that function exists.
HEADLAND_WEST = -round(GUTTER_HALF_LENGTH + HEADLAND_MARGIN, 3)
HEADLAND_EAST = +round(WALL_X - HEADLAND_MARGIN, 3)
HEADLANDS = (HEADLAND_WEST, HEADLAND_EAST)


def band(y: float) -> int | None:
    """Which free band contains y, or None if it is over a gutter."""
    for i, (lo, hi) in enumerate(BANDS):
        if lo <= y <= hi:
            return i
    return None


def route(sx: float, sy: float, tx: float, ty: float
          ) -> list[tuple[float, float]]:
    """Via points from the sensor point (sx, sy) to (tx, ty)."""
    here, there = band(sy), band(ty)
    if here is not None and here == there:
        return [(tx, ty)]
    # Both headlands work. Take the one that makes the trip shorter -- out
    # to it, across, and back in -- which is a comparison of two numbers,
    # not a search.
    end = min(HEADLANDS, key=lambda e: abs(sx - e) + abs(e - tx))
    return [(end, sy), (end, ty), (tx, ty)]


# ------------------------------------------------------------- validation
def footprint(x: float, y: float) -> tuple[float, float, float, float]:
    """(x0, y0, x1, y1) of the robot when its sensor point is at (x, y).

    Includes the boom, because the boom is 0.45 m up and the gutters are
    0.80 m tall: a boom over a gutter is a collision, not a near miss.
    """
    return (x - _SPAN_BACK, y - BASE_HALF_WIDTH,
            x + 0.0, y + BASE_HALF_WIDTH)


def _obstacles() -> list[tuple[float, float, float, float]]:
    return [(-GUTTER_HALF_LENGTH, y - GUTTER_HALF_WIDTH,
             +GUTTER_HALF_LENGTH, y + GUTTER_HALF_WIDTH)
            for y in ROW_Y.values()]


def clearance(x: float, y: float) -> float:
    """Smallest distance from the robot's rectangle to a gutter or a wall.

    Negative means it is inside one. Returned as a number rather than a
    boolean so a route can be reported as "0.08 m of margin" instead of
    "fine", which is the difference between a check that catches a change
    and one that only catches a disaster.
    """
    x0, y0, x1, y1 = footprint(x, y)
    margins = [WALL_X - x1, x0 + WALL_X, WALL_Y - y1, y0 + WALL_Y]
    for ox0, oy0, ox1, oy1 in _obstacles():
        # Separation along each axis; the rectangles miss if EITHER is
        # positive, so the clearance of a pair is the larger of the two.
        margins.append(max(ox0 - x1, x0 - ox1, oy0 - y1, y0 - oy1))
    return min(margins)


def leg_clearance(ax: float, ay: float, bx: float, by: float,
                  step: float = 0.02) -> float:
    """Worst clearance anywhere along a straight leg."""
    n = max(1, int(max(abs(bx - ax), abs(by - ay)) / step))
    return min(clearance(ax + (bx - ax) * i / n, ay + (by - ay) * i / n)
               for i in range(n + 1))


def route_clearance(sx: float, sy: float, tx: float, ty: float) -> float:
    """Worst clearance anywhere along the whole route."""
    worst, here = float("inf"), (sx, sy)
    for nxt in route(sx, sy, tx, ty):
        worst = min(worst, leg_clearance(*here, *nxt))
        here = nxt
    return worst
