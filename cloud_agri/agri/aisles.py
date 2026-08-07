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
#: The robot's outline in its OWN frame, relative to base_link:
#: (xmin, ymin, xmax, ymax). Deliberately asymmetric -- the boom reaches
#: SENSOR_OFFSET_X forward and the chassis BASE_HALF_LENGTH back -- because
#: treating it as symmetric is what makes a brake stop for a gutter the tail
#: passes 0.21 m clear of.
BODY_BOX = (-BASE_HALF_LENGTH, -BASE_HALF_WIDTH,
            +SENSOR_OFFSET_X, +BASE_HALF_WIDTH)


def brake_clearance(px: float, py: float, dx: float, dy: float,
                    margin: float = 0.03) -> float | None:
    """How far a lidar return is beyond the robot, along the way it is going.

    (px, py) is the return in the ROBOT's frame; (dx, dy) is the unit
    direction of travel, also in the robot's frame. Returns None when the
    point is not in the corridor the robot is about to sweep, and a negative
    number when it is inside the robot's own outline.

    WHY A CORRIDOR AND NOT A CONE

    A cone of +/-35 degrees was the first version, and it stopped the robot
    dead in every aisle: at 0.35 m of lateral clearance a gutter wall the
    robot is driving PARALLEL to sits well inside a 35-degree cone, so the
    brake fired on the very thing the route was designed to pass. A corridor
    asks the only question that matters -- is this in the way? -- and a wall
    you are driving alongside is not in the way.

    WHY THE OUTLINE IS PROJECTED RATHER THAN APPROXIMATED

    The corridor's edges are the extreme projections of the robot's own four
    corners onto the axis across the travel direction. For a symmetric robot
    a half-width would do; this one has a 0.50 m boom in front and 0.29 m of
    chassis behind, so when it STRAFES the corridor is 0.79 m wide and not
    centred on the base. Getting that wrong makes the robot brake at the east
    headland for the gutter end its tail clears by 0.21 m.

    WHY NEGATIVE MEANS IGNORE IT

    The lidar sits inside the robot and sees the robot: the camera pedestal
    and the mast both cross its plane, 0.12 and 0.22 m ahead. Those return
    ranges far shorter than any obstacle ever will, so a raw range test
    brakes permanently, from the first metre, for the robot's own body. A
    return inside the outline can only be the robot.
    """
    ex, ey = -dy, dx                       # across the direction of travel
    corners = [(BODY_BOX[0], BODY_BOX[1]), (BODY_BOX[0], BODY_BOX[3]),
               (BODY_BOX[2], BODY_BOX[1]), (BODY_BOX[2], BODY_BOX[3])]
    across = [cx * ex + cy * ey for cx, cy in corners]
    lateral = px * ex + py * ey
    if not (min(across) - margin <= lateral <= max(across) + margin):
        return None                        # beside the corridor, not in it
    front = max(cx * dx + cy * dy for cx, cy in corners)
    return (px * dx + py * dy) - front


#: How close a lidar return has to be to a wall or a gutter before it is
#: taken to BE that wall or gutter rather than something standing on it.
#: Wide enough for the odometry error and the beam's own resolution, far
#: narrower than anything the brake exists to stop for.
STRUCTURE_TOLERANCE = 0.12


def known_structure(wx: float, wy: float,
                    tol: float = STRUCTURE_TOLERANCE) -> bool:
    """Is this WORLD point part of the greenhouse itself?

    The brake exists for things that should not be there. The walls and the
    gutters should be there, they are in the catalogue, and route_clearance()
    has already proved every route passes them by at least 0.08 m. Braking
    for them is not caution, it is refusing to move.

    THIS IS THE FIX FOR TWO REAL FAILURES, AND THEY LOOKED DIFFERENT.

    Every leg to a headland stopped. The robot has to back its tail to
    within 0.08 m of the end wall to fit in the 0.95 m gap, and the brake
    wanted 0.35 m of clear floor behind it, so it refused at 0.27 m short
    and the mission logged "something is in the aisle". Something was: the
    building.

    And a leg that shifts sideways while going forward -- 0.10 m across
    while travelling 0.66 m along, which is what moving between the two
    stations of an inner aisle looks like -- tilts the swept corridor by
    8.6 degrees and swings it onto the gutter the robot is driving BESIDE.
    Same wall, different geometry, same wrong answer.

    Neither is an obstacle. Both are on the map.
    """
    if abs(wx) >= WALL_X - tol or abs(wy) >= WALL_Y - tol:
        return True
    return any(ox0 - tol <= wx <= ox1 + tol and oy0 - tol <= wy <= oy1 + tol
               for ox0, oy0, ox1, oy1 in _obstacles())


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
