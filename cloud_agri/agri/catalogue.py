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

#: WHICH POINT OF THE ROBOT GOES ON THE CROSS.
#:
#: Not base_link. A robot that has a body cannot see the floor under its own
#: middle -- the chassis is in the way -- so a marker under the centre can be
#: driven to but never verified. The reference point is therefore the SENSOR
#: HEAD, on a short boom over the front bumper, with the floor camera looking
#: straight down at it. "The cross is in the middle of the picture" and "the
#: sensor is on the cross" then mean the same thing, with no lever arm to get
#: the sign of.
#:
#: 0.50 m is the smallest offset that also keeps the camera mast out of the
#: downward view cone (the mast reaches x = 0.27 at 0.45 m up; a camera at
#: 0.50 m sees the floor from x = 0.27 outwards, so no ray crosses it).
#:
#: Consequences, stated once: the base centre stands 0.50 m back along the
#: aisle from the station; every pose in a report is the SENSOR point, not
#: base_link; and the robot always drives with yaw = 0, so the offset is
#: (+0.50, 0) in the world too and never needs rotating.
SENSOR_OFFSET_X = 0.50

#: WHERE THE PAN HEAD SITS, and why it is not where the cross is.
#:
#: The pan mast is mounted 0.18 m ahead of base_link (j_camera_pan in the
#: URDF). The sensor point -- the boom tip, under the floor camera, the
#: point a station names -- is 0.50 m ahead of it. So the lens that
#: photographs the plant is CAMERA_LEVER_X = 0.32 m BEHIND the cross the
#: robot is standing on.
#:
#: That distance is small and its consequence is not. Aiming the head with
#: the bearing from the CROSS rather than from the LENS misses by 30.2
#: degrees at every station, which against a 69-degree field of view puts
#: the plant hard against the edge of the frame -- present in every
#: photograph, centred in none, and never obviously broken.
#:
#: make_robot.py asserts this equals the joint origin it writes, so the
#: number cannot drift away from the robot it describes.
CAMERA_PIVOT_X = 0.18
CAMERA_LEVER_X = SENSOR_OFFSET_X - CAMERA_PIVOT_X

# ------------------------------------------------ framing the photograph
#: Where a plant stands, and how tall the tallest of the four meshes is.
#: PLANT_BASE_Z is the gutter top; set it to 0.0 the day the plants are
#: grown on the floor and every number below follows without being retuned.
PLANT_BASE_Z = 0.80

#: How tall the tallest installed mesh stands once fitted to
#: PLANT_WIDTH_M. Measured, not guessed -- the suite asserts it against
#: agri.world.plants.fitted_height() so it cannot drift from the assets.
#: A strawberry is a rosette: wider than it is tall.
PLANT_HEIGHT_M = 0.15

#: The pan camera's height above the floor. Phase B's, read back from the
#: URDF by make_robot so it cannot drift away from the robot it describes.
PLANT_CAM_Z = 0.78

#: How wide a plant is across the leaves. Must match
#: agri.world.plants.PLANT_WIDTH_M, which is what the meshes are scaled
#: to; the suite asserts the two agree. Duplicated rather than imported so
#: that the catalogue -- which everything depends on -- depends on nothing.
PLANT_SPREAD_M = 0.27

#: How much of the frame the plant should fill, in its LARGER dimension.
#:
#: Not 1.0, and not 0.3 either. Too tight and a 4 cm parking error -- the
#: tolerance -- crops a leaf off; too loose and the picture is mostly
#: greenhouse. 0.55 leaves about 9 degrees of horizontal margin, against
#: the 3.6 degrees that the worst allowed parking error is worth.
PLANT_FRAME_FILL = 0.55

#: The image the Phase B camera produces. Only the RATIO matters here.
PLANT_CAM_W, PLANT_CAM_H = 640, 480


def plant_camera_distance() -> float:
    """Lens to plant centre, horizontally, at a parked station.

    Two offsets at right angles: the lens sits CAMERA_LEVER_X behind the
    sensor point along the aisle, and the plant is STATION_REACH across it.
    """
    return math.hypot(CAMERA_LEVER_X, STATION_REACH)


def plant_camera_frame() -> tuple[float, float]:
    """(pitch, horizontal_fov) in radians, for the plant camera.

    WHY THIS IS COMPUTED AND NOT TUNED

    Phase B's camera looks 16 degrees UP through a 68.8 degree lens,
    because it was aimed at crates down an aisle. Pointed at a 27 cm plant
    0.64 m away, that lens sees 87 x 65 cm: the plant is 31 % of the width
    and 43 % of the height, and the remaining two thirds of every one of
    the 48 photographs is wall, floor and sky. The first real photograph
    made that obvious in a way no amount of geometry checking had.

    Pitch is NEGATIVE for nose-up, following the URDF's own convention
    (rotation about +y takes +x downward), which is why the sign here looks
    backwards and is not.

    Both numbers come out of PLANT_BASE_Z, so moving the plants -- onto the
    floor, or onto a taller gutter -- retargets the camera by itself. That
    is the whole reason this is a function.
    """
    d = plant_camera_distance()
    bottom = math.atan2(PLANT_BASE_Z - PLANT_CAM_Z, d)
    top = math.atan2(PLANT_BASE_Z + PLANT_HEIGHT_M - PLANT_CAM_Z, d)

    # Centre on the plant, not on its base: aiming at the pot puts the
    # canopy -- the part worth photographing -- in the top of the frame.
    pitch = -(bottom + top) / 2.0

    # SIZED ON THE LARGER DIMENSION, WHICH IS THE WIDTH.
    #
    # A strawberry is a rosette: 27 cm across and 15 cm tall. Sizing the
    # lens on the height alone filled 63 % of the frame vertically and
    # 85 % horizontally, which leaves less margin than one parking error
    # is worth -- the first version did exactly that and would have
    # cropped a leaf off any station parked 4 cm wide.
    #
    # So: pick the vertical field that satisfies BOTH, given the image's
    # aspect ratio, and take whichever is larger.
    aspect = PLANT_CAM_H / PLANT_CAM_W
    want = max(PLANT_HEIGHT_M, PLANT_SPREAD_M * aspect)
    vfov = 2.0 * math.atan(want / (2.0 * d * PLANT_FRAME_FILL))
    hfov = 2.0 * math.atan(math.tan(vfov / 2.0) / aspect)
    return pitch, hfov


def plant_frame_span() -> tuple[float, float, float, float]:
    """(frame bottom, plant bottom, plant top, frame top), in radians.

    Returned so the suite can assert the plant is inside the frame with
    margin, rather than trusting the arithmetic above.
    """
    d = plant_camera_distance()
    pitch, hfov = plant_camera_frame()
    vfov = 2.0 * math.atan(math.tan(hfov / 2.0) * PLANT_CAM_H / PLANT_CAM_W)
    return (-pitch - vfov / 2.0,
            math.atan2(PLANT_BASE_Z - PLANT_CAM_Z, d),
            math.atan2(PLANT_BASE_Z + PLANT_HEIGHT_M - PLANT_CAM_Z, d),
            -pitch + vfov / 2.0)


#: HOW FAR PAST THE PLANT THE CROSS IS PAINTED, along the aisle.
#:
#: The robot cannot see the floor under its own middle -- the chassis is in
#: the way -- which is the whole reason the boom exists. So the cross has to
#: be under the boom tip, and the question is where that leaves the rest of
#: the robot.
#:
#: With the cross at the plant's own x, the chassis ends up 0.50 m down the
#: aisle from the plant it is measuring: the robot is demonstrably NOT at
#: the plant, in Gazebo and in every photograph. Offsetting the cross by
#: exactly the boom length puts base_link on the plant's x instead. The
#: robot then straddles the plant, the floor camera still lies over the
#: cross, and the pan head has only the 0.18 m mast offset left to correct.
STATION_ALONG_AISLE = SENSOR_OFFSET_X

#: Half-length of a painted cross arm, and the thickness of the arms, in
#: metres. The arm length is NOT a matter of taste: in an inner aisle two
#: stations sit at the SAME x, 0.10 m apart in y, so their y arms lie on one
#: line. Any arm longer than 0.05 makes the two crosses touch, and a red
#: blob detector then sees ONE marker straddling both -- it would centre the
#: robot on the midpoint, 0.05 m from either station, and file the visit
#: under whichever label it guessed.
#:
#: 0.04 leaves a 0.02 m gap, about ten pixels for the floor camera, which is
#: enough for the connected-component step to keep them apart. The first
#: version of this file used 0.09 and carried a comment claiming the crosses
#: did not merge; rendering the pair and running the detector over it is
#: what showed the comment was wrong.
CROSS_ARM = 0.04
CROSS_THICKNESS = 0.02
#: The smallest gap the detector may be left with. Asserted by the test
#: suite against the real station spacing, so this cannot silently rot.
MIN_CROSS_GAP = 0.015


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
    def cross(self) -> tuple[float, float]:
        """Where the painted mark is -- UNDER THE PARKED CHASSIS.

        Not where the robot's sensor point ends up. The mark sits at the
        base centre, so a parked robot covers it completely, which is what
        a floor fiducial looks like on every AGV that uses one and is what
        makes the simulation read as a robot standing ON its marker rather
        than stopped short of it.

        The floor camera is at the boom tip, SENSOR_OFFSET_X ahead of the
        base, so at the park pose it is looking at empty floor 0.50 m past
        the mark. The mark is therefore read on the WAY IN: the robot stops
        with its boom over it, corrects there, and then advances the boom
        length so the chassis comes to rest on it. See Driver.drive_to.
        """
        return (self.x - SENSOR_OFFSET_X, self.y)

    @property
    def fix_pose(self) -> tuple[float, float]:
        """Where the SENSOR POINT must be for the camera to see the mark.

        The same point as the mark, by construction -- the camera is the
        sensor point. Named separately because the driver reasons about it
        as a waypoint and confusing it with the park pose is the one
        mistake that would make the whole sequence silently pointless.
        """
        return self.cross

    def camera_position(self, robot_yaw: float = 0.0) -> tuple[float, float]:
        """Where the PAN HEAD is when the robot is parked on this cross.

        Not the cross. The cross is under the floor camera at the boom tip;
        the pan head is on the mast, CAMERA_LEVER_X behind it along the
        robot's own +x axis. Those are two different points on one rigid
        body, and aiming from the wrong one is what this method exists to
        stop -- see pan_for.
        """
        return (self.x - CAMERA_LEVER_X * math.cos(robot_yaw),
                self.y - CAMERA_LEVER_X * math.sin(robot_yaw))

    @property
    def plant_bearing(self) -> float:
        """World angle from the PAN HEAD to the plant, driving east (yaw=0).

        Positive when the plant is to the north (+y). The robot subtracts
        its own heading from this to get the pan angle, so the same station
        works whichever way down the aisle it is driving. Storing a fixed
        robot yaw here instead would silently assume a direction of travel,
        and half the visits would look the wrong way.
        """
        cx, cy = self.camera_position(0.0)
        return math.atan2(self.plant_y - cy, self.plant_x - cx)

    def pan_for(self, robot_yaw: float) -> float:
        """Head angle that points the camera at the plant from this station.

        MEASURED FROM THE PAN HEAD, NOT FROM THE CROSS. This was the bug
        that made every photograph useless without making any of them
        obviously wrong: the angle was computed from the station -- the boom
        tip -- while the lens sits CAMERA_LEVER_X = 0.32 m behind it. The
        two points see the plant 30.2 degrees apart, and with a 69-degree
        field of view that is 0.9 of a half-frame: the plant sat at the very
        edge of every image, in frame, so nothing looked broken.
        """
        cx, cy = self.camera_position(robot_yaw)
        d = math.atan2(self.plant_y - cy, self.plant_x - cx) - robot_yaw
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
                   x=px + STATION_ALONG_AISLE, y=py + sign * STATION_REACH,
                   plant_x=px, plant_y=py)


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
