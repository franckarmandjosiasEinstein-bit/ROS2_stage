"""survey -- in what order to visit a set of stations, and what it costs.

WHY THIS EXISTS

The Cloud emits stations in SURVEY order: P1,1R, P1,1L, P1,2R, P1,2L, ...
That order is right for a human reading a list and wrong for a robot driving
a greenhouse, because the two sides of one plant are not next to each other
in the building. R is south of the row, L is north of it, and between them
lies a 0.40 m gutter that is 8 m long. Going from P1,1R to P1,1L means
driving to the end of the row, crossing, and coming back -- and the survey
order asks for that forty-five times.

Measured against the real geometry, over all 48 stations from the dock:

    survey order, as issued          268.6 m      45 gutter crossings
    swept by band                     40.4 m       3 gutter crossings

228 m, at 0.45 m/s, is eight and a half minutes of a battery spent arriving
at places the robot had already been next to. The mission that produced the
first recorded campaign took 1505 s; the driving in it was most of that.

WHY BY BAND

aisles.py already divides the free space into four bands running along x.
Every station sits in exactly one, and the bands are what the gutters
separate:

    band 0   P1,*R                      8 stations
    band 1   P1,*L and P2,*R           16 stations
    band 2   P2,*L and P3,*R           16 stations
    band 3   P3,*L                      8 stations

Within a band the robot drives freely along x; leaving one costs a trip to a
headland whatever the destination. So the cheap route is the obvious one --
sweep a band end to end, cross once, sweep the next one back -- and the only
real decisions are which end to start at and which way to go. There are four
combinations. They are all four evaluated, exactly, against the same route()
the driver will actually follow.

WHY THE ORDER AS ISSUED IS ALSO A CANDIDATE

Because then this can never make a mission longer than not having it. The
five candidates are measured and the shortest wins, with the issued order
winning ties. That is a property worth having by construction rather than by
argument: a request for two stations in the same band does not need
reordering, and here it provably does not get any.

WHAT THIS DELIBERATELY IS NOT

A TSP solver. The greenhouse is four parallel corridors joined at the ends;
the optimal tour of a set of points on a line is to walk the line. A solver
would spend milliseconds rediscovering that, and would then be free to
return something else the day a constant changes. Four candidates and a
measurement are honest about how little search this problem needs.
"""

from __future__ import annotations

import math

from agri.aisles import band, route
from agri.catalogue import by_label

# --------------------------------------------------------------- geometry
def leg_lengths(labels, sx: float, sy: float) -> list[float]:
    """Length of each leg the driver will fly, in metres, in order.

    Uses aisles.route() -- the same via points drive_to() follows -- and
    measures each leg straight, because _drive() sends the base at the
    waypoint along the vector to it rather than along the axes.
    """
    stations = by_label()
    out: list[float] = []
    x, y = sx, sy
    for lab in labels:
        s = stations[lab]
        for wx, wy in route(x, y, s.x, s.y):
            out.append(math.hypot(wx - x, wy - y))
            x, y = wx, wy
    return out


def path_length(labels, sx: float, sy: float) -> float:
    """Total metres driven to visit `labels` in that order from (sx, sy)."""
    return sum(leg_lengths(labels, sx, sy))


def crossings(labels, sx: float, sy: float) -> int:
    """How many times the route leaves one band for another.

    The expensive event, counted on its own: every one is a trip out to a
    headland and back, and it is the number that changes when the order does.
    """
    stations = by_label()
    here, n = band(sy), 0
    for lab in labels:
        there = band(stations[lab].y)
        if there != here:
            n += 1
        here = there
    return n


# ---------------------------------------------------------------- ordering
def _by_band(labels, stations) -> dict[int, list[str]]:
    groups: dict[int, list[str]] = {}
    for lab in labels:
        b = band(stations[lab].y)
        if b is None:                # a station over a gutter cannot exist,
            continue                 # but do not lose it if one ever does
        groups.setdefault(b, []).append(lab)
    return groups


def _sweep(labels, stations, bands_descending: bool, start_east: bool
           ) -> list[str]:
    """One boustrophedon: bands in order, direction flipping between them."""
    groups = _by_band(labels, stations)
    out: list[str] = []
    east = start_east
    for b in sorted(groups, reverse=bands_descending):
        # (x, y) rather than x alone: band 1 and band 2 each hold two
        # stations at the SAME x, 0.10 m apart across the aisle. Without the
        # tie-break their order would come out of dict insertion and change
        # the measured length for no reason anybody could see.
        out += sorted(groups[b], key=lambda l: (stations[l].x, stations[l].y),
                      reverse=east)
        east = not east
    # Stations that are somehow in no band keep their issued order, at the
    # end, rather than vanishing from the mission.
    placed = set(out)
    return out + [l for l in labels if l not in placed]


def survey_order(labels, sx: float, sy: float) -> list[str]:
    """Reorder `labels` to shorten the drive, starting from (sx, sy).

    Never returns anything longer than the order it was given: see the five
    candidates in the module docstring. The set is preserved exactly -- this
    reorders a mission, it never drops or adds a station.
    """
    labels = list(labels)
    if len(labels) < 3:              # nothing to reorder that could help
        return labels
    stations = by_label()
    known = [l for l in labels if l in stations]
    if len(known) != len(labels):    # an unknown label has no position; the
        return labels                # driver will refuse it and say so
    candidates = [labels]
    for descending in (False, True):
        for east in (False, True):
            candidates.append(_sweep(labels, stations, descending, east))
    # min() keeps the FIRST minimum, and the issued order is first, so an
    # order that saves nothing is not adopted.
    return min(candidates, key=lambda c: path_length(c, sx, sy))


# ------------------------------------------------------------------ energy
#: THESE FOUR NUMBERS ARE ASSUMPTIONS, NOT MEASUREMENTS.
#:
#: Gazebo models no battery, this robot has no current sensor, and inventing
#: a percentage that ticks down would put a fabricated number next to forty
#: eight real ones. So the distance is MEASURED -- odometry, integrated, and
#: reported per mission -- and the energy is DERIVED from it by a model whose
#: constants are written here, in the open, for someone to replace with a
#: bench measurement.
#:
#: The shape of the model is the part worth trusting: a platform draws a
#: roughly constant idle power whether or not it is moving, and the drive
#: adds to it only while moving. That is why shortening the route helps less
#: than the 85 % figure suggests -- it removes drive energy, not idle energy
#: -- and saying so is more useful to a report than a bigger number.
IDLE_W = 18.0        # computer, lidar, two cameras, MQTT link
DRIVE_W = 45.0       # the four wheel motors, while actually moving
CRUISE_MPS = 0.45    # driver.V_MAX; what a long leg settles at
BATTERY_WH = 120.0   # 24 V, 5 Ah -- the youbot's nameplate pack


def energy_wh(metres: float, seconds: float) -> dict[str, float]:
    """Estimated energy for a mission of `metres` driven over `seconds`.

    Returns the breakdown rather than one number, because the split is the
    finding: on a 48-station sweep the robot spends far longer standing at
    crosses -- settling, focusing, photographing, encoding a QR -- than it
    spends driving between them, and idle draw dominates. Shortening the
    route attacks the smaller half.
    """
    driving_s = min(seconds, metres / CRUISE_MPS) if CRUISE_MPS > 0 else 0.0
    idle = IDLE_W * seconds / 3600.0
    drive = DRIVE_W * driving_s / 3600.0
    total = idle + drive
    return {"idle_wh": round(idle, 2), "drive_wh": round(drive, 2),
            "total_wh": round(total, 2),
            "driving_s": round(driving_s, 1),
            "battery_pct": round(100.0 * total / BATTERY_WH, 1)}
