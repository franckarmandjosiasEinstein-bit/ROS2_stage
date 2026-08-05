"""labels -- the station identifier Pi,jS, parsed and formatted in one place.

    P2,5R   row 2, plant 5, RIGHT-hand side
    P1,8L   row 1, plant 8, LEFT-hand side

WHY A WHOLE MODULE FOR A STRING

Because four independent programs have to agree on it: the world generator
that paints the red cross, the robot that drives to it, the Cloud that asks
for it, and the dashboard that displays it. The moment two of them parse the
label slightly differently -- accepting "P2,5 R" here and rejecting it there,
or disagreeing on whether rows count from 0 -- the system fails in a way that
looks like a network fault. One parser, one formatter, one set of bounds.

WHICH SIDE IS RIGHT

Stated once, here, because it is the only part of the label that is a
convention rather than a number, and a convention nobody wrote down is a
convention two people will read differently.

    Rows run along the x axis. Stand at the start of a row (small x) and
    look along it, toward +x. Your LEFT hand points to +y. So:

        L (left)  = the station on the +y side of the plant
        R (right) = the station on the -y side of the plant

This matches the ordinary reading of "left/right of the row" for someone
walking down it, and it is fixed to the WORLD, not to the robot: a robot
driving back down the row in the -x direction still collects P2,5R at the
same physical place. Tying the side to the robot's heading instead would
make the label depend on the direction of travel, and the same plant would
be filed under two names.
"""

from __future__ import annotations

import re

ROWS = 3                 # i, 1..3
PLANTS_PER_ROW = 8       # j, 1..8
SIDES = ("R", "L")

#: Canonical form: P<row>,<plant><SIDE>, e.g. "P2,5R".
_RE = re.compile(r"^\s*P\s*(\d+)\s*[,;.\-_ ]\s*(\d+)\s*([RL])\s*$", re.I)


class LabelError(ValueError):
    """A label that is not a station. Carries what was wrong, not just that
    something was: the Cloud echoes this straight back to the operator."""


def format_label(row: int, plant: int, side: str) -> str:
    """Canonical label. Validates, so a bad one cannot be built by accident."""
    side = side.upper()
    if not 1 <= row <= ROWS:
        raise LabelError(f"row {row} is outside 1..{ROWS}")
    if not 1 <= plant <= PLANTS_PER_ROW:
        raise LabelError(f"plant {plant} is outside 1..{PLANTS_PER_ROW}")
    if side not in SIDES:
        raise LabelError(f"side {side!r} is not R or L")
    return f"P{row},{plant}{side}"


def parse_label(text: str) -> tuple[int, int, str]:
    """(row, plant, side) from a label, tolerant of how a human types it.

    Accepts "P2,5R", "p2, 5 r", "P2-5L". Refuses anything else, and refuses
    out-of-range numbers even when the shape is right -- "P4,1R" parses
    cleanly and names a row that does not exist, which is exactly the request
    that would otherwise send the robot to a computed position in a wall.
    """
    m = _RE.match(text or "")
    if not m:
        raise LabelError(
            f"{text!r} is not a station label. Expected P<row>,<plant><R|L>, "
            "for example P2,5R")
    row, plant, side = int(m.group(1)), int(m.group(2)), m.group(3).upper()
    # Validate HERE, not only in normalise(). catalogue.station() parses a
    # label and computes a position from the numbers; if the range check
    # lived only in the formatter, "P4,1R" would quietly yield coordinates
    # inside the north wall and the robot would drive at it.
    format_label(row, plant, side)
    return row, plant, side


def normalise(text: str) -> str:
    """Any accepted spelling -> the canonical one. Used on every request, so
    the store is never keyed by two spellings of the same station."""
    return format_label(*parse_label(text))


#: A PLANT, with no side: "P2,5". Same shape as a station label minus the
#: R/L, and it names two stations rather than one.
_RE_PLANT = re.compile(r"^\s*P\s*(\d+)\s*[,;.\-_ ]\s*(\d+)\s*$", re.I)


def expand_target(text: str) -> list[str]:
    """One thing a human or a Cloud asked for -> the stations it names.

        "P2,5R"  ->  ["P2,5R"]              one side of one plant
        "P2,5"   ->  ["P2,5R", "P2,5L"]     the PLANT, both sides

    The second form exists because the brief is written in terms of plants:
    the Cloud asks for the data of a given plant, or of all of them. A plant
    is not a station -- it has a measurement point on each side of it -- so a
    system that could only be asked for "P2,5R" would be answering a
    different question from the one it was given, and the operator would have
    to know to ask twice.

    Order is R then L, matching all_labels(), so a request for a plant and a
    sweep of the greenhouse visit the same two points in the same order.
    """
    stripped = (text or "").strip()
    m = _RE_PLANT.match(stripped)
    if m:
        row, plant = int(m.group(1)), int(m.group(2))
        return [format_label(row, plant, side) for side in SIDES]
    return [normalise(stripped)]


def all_labels() -> list[str]:
    """Every station, in survey order: row by row, plant by plant, R then L.

    The order is not arbitrary. It is the order the robot would naturally
    visit if it simply worked through the list, and the Cloud uses it as the
    default plan for a whole-greenhouse sweep.
    """
    return [format_label(i, j, s)
            for i in range(1, ROWS + 1)
            for j in range(1, PLANTS_PER_ROW + 1)
            for s in SIDES]


def station_count() -> int:
    return ROWS * PLANTS_PER_ROW * len(SIDES)
