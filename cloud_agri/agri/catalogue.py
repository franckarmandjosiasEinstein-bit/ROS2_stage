"""catalogue -- where every plant and every red cross is, in metres.

THE SINGLE SOURCE OF TRUTH

Three programs need these coordinates and they must not disagree:

    agri/world/make_world.py   paints the red crosses on the greenhouse floor
    ros2/src/agri_robot       drives the robot onto them
    agri/cloud                draws them on the dashboard map

The obvious way to build this is to type the numbers into the SDF and then
type them again into the robot. That is how a cross ends up painted at
y = -1.70 and driven to at y = -1.65, and the robot then reports it is "on"
a cross it is 5 cm away from, with nothing anywhere to catch it. So the
numbers exist once, here, and the world is GENERATED from them.

GEOMETRY, AND WHY THESE NUMBERS

The greenhouse is the one already built and validated in Phase B: interior
9.90 x 4.90 m, wall inner faces at x = +/-4.95 and y = +/-2.45, three plant
rows on gutters 0.40 m wide centred at y = -1.20, 0.00, +1.20, and eight
plants per row at x = -3.15 .. +3.15 in steps of 0.90 m. The regression test
reads those positions back out of the Phase B world, so this file cannot
drift away from the greenhouse it claims to describe.

THE ROBOT STANDS ALONG THE AISLE, IT DOES NOT TURN TO FACE THE PLANT

This is the one decision here that is not forced by the building, and the
first version got it wrong, so it is worth writing down.

Turning the base to face the plant points its LONG axis across the aisle:
0.29 m of half-length toward the gutter instead of 0.19 m of half-width. An
inner aisle is only 0.80 m wide between two gutter edges, so a base turned
across it has 0.22 m of slack in total -- and the two stations that share
that aisle then cannot both fit with any usable margin. The first attempt
left 1 cm.

Keeping the base ALONG the aisle and letting the pan head look sideways --
which is what the head is for, and what it already does -- turns 0.29 into
0.19 and buys back 0.20 m. Every station then has at least 0.16 m of
clearance. The camera ends up in the same place either way; only the
chassis is oriented differently.

STATION_REACH is then bounded on both sides and there is not much room:

    too small  the body overlaps the gutter (needs reach > 0.19 + 0.20)
    too large  the two stations sharing an inner aisle push each other out,
               and the outer stations approach the side wall

0.55 m leaves 0.16 m of clearance at every one of the 48 stations. The
regression test asserts that, so changing the reach cannot silently put a
wheel on a gutter.

The price is that the two stations sharing an inner aisle -- P1,jL and
P2,jR, say -- sit only 0.10 m apart. That is real and it is why the crosses
are painted and detected visually rather than merely driven to on odometry:
0.10 m is inside the error a drifting estimate accumulates in one lap.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from agri.labels import (PLANTS_PER_ROW, ROWS, SIDES, format_label,
                         parse_label)

# --- the greenhouse, as built -------------------------------------------
INTERIOR_X = 9.90              # m, wall inner face to wall inner face
INTERIOR_Y = 4.90
WALL_X = 4.95                  # inner face of the end walls
WALL_Y = 2.45                  # inner face of the side walls

ROW_Y = {1: -1.20, 2: 0.00, 3: +1.20}    # plant row (gutter) centres
GUTTER_HALF_WIDTH = 0.20                  # the gutter is 0.40 m wide
PLANT_X0, PLANT_DX = -3.15, 0.90          # first plant, then spacing along x

# --- the stations -------------------------------------------------------
STATION_REACH = 0.55           # m from the plant centre, across the row
BASE_HALF_LENGTH = 0.29        # the youbot footprint, for the clearance check
BASE_HALF_WIDTH = 0.19

#: Side -> sign of the y offset. See labels.py for why L is +y.
SIDE_SIGN = {"L": +1.0, "R": -1.0}

#: Radius of the painted cross, and the thickness of its arms, in metres.
#: Big enough for the camera to resolve from a couple of metres, small
#: enough that two crosses 0.10 m apart do not merge into one blob.
CROSS_ARM = 0.09
CROSS_THICKNESS = 0.025


@dataclass(frozen=True)
class Station:
    """One red cross, and everything anybody needs to know about it."""

    label: str          # "P2,5R"
    row: int            # 1..3
    plant: int          # 1..8
    side: str           # "R" | "L"
    x: float            # where the robot must stand (the cross)
    y: float
    plant_x: float      # the plant to measure and photograph
    plant_y: float

    @property
    def plant_bearing(self) -> float:
        """World angle from the cross to the plant, in radians.

        +pi/2 when the plant is to the north (+y), -pi/2 when south. The
        robot subtracts its own heading from this to get the pan angle, so
        the same station works whichever way down the aisle it is driving.
        Storing a fixed robot yaw here instead would silently assume a
        direction of travel, and half the visits would look the wrong way.
        """
        return math.atan2(self.plant_y - self.y, self.plant_x - self.x)

    def pan_for(self, robot_yaw: float) -> float:
        """Head angle that points the camera at the plant from this station."""
        d = self.plant_bearing - robot_yaw
        return math.atan2(math.sin(d), math.cos(d))

    @property
    def distance_to_plant(self) -> float:
        return math.hypot(self.plant_x - self.x, self.plant_y - self.y)


def plant_position(row: int, plant: int) -> tuple[float, float]:
    """Centre of plant Pi,j, in metres."""
    format_label(row, plant, "R")          # validates the ranges, cheaply
    return PLANT_X0 + (plant - 1) * PLANT_DX, ROW_Y[row]


def station(label_or_row, plant: int | None = None,
            side: str | None = None) -> Station:
    """The Station for a label ("P2,5R") or for (row, plant, side)."""
    if plant is None and side is None:
        row, plant, side = parse_label(str(label_or_row))
    else:
        row = int(label_or_row)
    label = format_label(row, plant, side)      # validates
    px, py = plant_position(row, plant)
    sign = SIDE_SIGN[side.upper()]
    return Station(label=label, row=row, plant=plant, side=side.upper(),
                   x=px, y=py + sign * STATION_REACH, plant_x=px, plant_y=py)


def all_stations() -> list[Station]:
    """All 48, in survey order (see labels.all_labels)."""
    return [station(i, j, s)
            for i in range(1, ROWS + 1)
            for j in range(1, PLANTS_PER_ROW + 1)
            for s in SIDES]


def by_label() -> dict[str, Station]:
    return {s.label: s for s in all_stations()}


def nearest_station(x: float, y: float) -> tuple[Station, float]:
    """The station closest to (x, y), and how far it is.

    Answers 'which cross am I standing on?' from the pose alone, without
    trusting the mission's own idea of where it meant to go -- which is the
    only way to catch a visit that was filed under the wrong label.
    """
    best = min(all_stations(), key=lambda s: math.hypot(s.x - x, s.y - y))
    return best, math.hypot(best.x - x, best.y - y)


def clearances(s: Station) -> tuple[float, float]:
    """(nearest gutter, nearest wall) clearance of the footprint, in metres.

    The base stands ALONG the aisle, so its half-WIDTH points across at the
    gutters. Using the half-length here instead -- the natural mistake, and
    the one the first version made -- under-reports the tight clearance by
    0.10 m, which is most of the margin there is.
    """
    lo, hi = s.y - BASE_HALF_WIDTH, s.y + BASE_HALF_WIDTH
    gutter = min(min(abs(lo - e), abs(hi - e))
                 for c in ROW_Y.values()
                 for e in (c - GUTTER_HALF_WIDTH, c + GUTTER_HALF_WIDTH))
    wall = WALL_Y - max(abs(lo), abs(hi))
    return gutter, wall


def worst_clearance() -> tuple[Station, float]:
    """The tightest station in the greenhouse, and by how much. Printed by
    the world generator so the number is in front of you when you change
    STATION_REACH, not buried in a test."""
    worst = min(all_stations(), key=lambda s: min(clearances(s)))
    return worst, min(clearances(worst))
