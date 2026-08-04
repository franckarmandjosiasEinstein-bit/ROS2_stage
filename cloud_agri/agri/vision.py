"""vision -- see the red cross on the floor and say where it is.

WHY THERE IS A SECOND CAMERA

The brief says the robot must position itself EXACTLY on the red points. The
Phase B camera cannot check that: it sits at 0.78 m pitched 16 degrees UP,
because its job is to look over a 0.8 m gutter at the fruit on top. The floor
first enters its field of view about four metres ahead, foreshortened to a
few pixels. Asking it to centre the robot on a marker under its own wheels is
asking the wrong sensor.

So the generated robot (agri/world/make_robot.py) carries a second, cheap
camera looking straight down -- the same thing a warehouse AGV uses to dock
on a floor marker. Straight down is the whole point: the patch of floor it
sees is centred on the SENSOR REFERENCE POINT (catalogue.SENSOR_OFFSET_X,
the boom over the front bumper), so "the cross is in the middle of the
image" and "the sensor is on the cross" are the same statement, with no
lever arm to subtract and no heading to get wrong.

Every offset this module returns is relative to that reference point, never
to base_link. The distinction is the one thing here worth remembering: it is
also why the chassis is not in the picture.

WHY THE MARKER IS A CROSS

Two arms at right angles, so the same blob gives three numbers instead of
two: where it is, and which way it is turned. Orientation comes out of the
FOUR-FOLD symmetry of the shape (see cross_yaw below), which a disc does not
have and a square gives less cleanly. It also gives a shape test that a red
sticker, a dropped tomato or a reflection fails: a cross fills under half of
its own bounding box, a solid blob fills all of it.

WHAT THIS MODULE DELIBERATELY DOES NOT DO

Talk to ROS. It takes an H x W x 3 uint8 array and returns numbers, so the
whole thing is tested against synthetic frames rendered by its own inverse
projection -- which is the only reason there is any evidence at all that the
signs are right, given that neither ROS nor Gazebo runs on the machine this
was written on.

THE SIGN CONVENTION, ONCE

A Gazebo camera looks along its own +x; image RIGHT is -y and image DOWN is
-z of the sensor frame. Mounted with rpy = (0, +pi/2, 0) on the base, the
sensor axes in BODY coordinates are

    x_s = (0, 0, -1)     straight down, the viewing direction
    y_s = (0, 1, 0)      body left
    z_s = (1, 0, 0)      body forward

so image up is body FORWARD and image right is body RIGHT, which is what
anyone reading the picture would assume. Everything below follows from those
three vectors and nothing re-derives them.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from agri.catalogue import CROSS_ARM, CROSS_THICKNESS, SENSOR_OFFSET_X

#: Mounting of the floor camera and its optics. These are the numbers
#: make_robot.py writes into the URDF; there is one copy of them, here.
#: CAM_X is measured from base_link and equals SENSOR_OFFSET_X: the camera
#: hangs directly over the reference point, which is what makes the image
#: centre mean "on the cross".
CAM_X, CAM_Y, CAM_Z = SENSOR_OFFSET_X, 0.0, 0.45
CAM_WIDTH, CAM_HEIGHT = 320, 240
CAM_HFOV = 1.2                       # rad, same lens as the Phase B camera

#: A pixel is red when it dominates both other channels by this factor. The
#: aisle paint is dark green (0.15, 0.25, 0.21) and the gutters are near
#: white, so the margin is wide; 1.6 rejects both without tuning.
RED_DOMINANCE = 1.6
RED_MIN = 90                         # 0..255, rejects red-ish shadow
#: A cross fills a fraction of its own bounding box:
#:     (2*arm*thickness*2 - thickness^2) / (2*arm)^2 = 0.44 as built
#: Accept a wide band around that -- the point is to reject a SOLID blob
#: (ratio 1.0), not to measure the marker. The band is wide on purpose: the
#: exact ratio moves with the arm and thickness in catalogue.py, and a
#: detector that had to be retuned whenever the paint changed would be
#: retuned wrong.
FILL_MIN, FILL_MAX = 0.10, 0.60
#: Below this the blob is noise, and its centroid is not worth a correction.
#: The marker is 0.08 m across and the camera resolves 520 px/m, so a real
#: one is about 740 pixels -- six times this floor.
MIN_PIXELS = 120


@dataclass(frozen=True)
class NadirCamera:
    """A camera looking straight down at the floor from the sensor boom."""

    height: float = CAM_Z
    width: int = CAM_WIDTH
    hpix: int = CAM_HEIGHT
    hfov: float = CAM_HFOV
    #: Where the camera sits relative to the REFERENCE POINT -- zero, by
    #: construction. Kept as a field so a mis-mounted camera can be
    #: described rather than silently absorbed into the readings.
    offset_x: float = 0.0
    offset_y: float = 0.0

    @property
    def focal(self) -> float:
        return (self.width / 2.0) / math.tan(self.hfov / 2.0)

    @property
    def centre(self) -> tuple[float, float]:
        return (self.width - 1) / 2.0, (self.hpix - 1) / 2.0

    def ground_to_pixel(self, dx: float, dy: float) -> tuple[float, float]:
        """Ground offset from the SENSOR REFERENCE POINT -> (u, v)."""
        cx, cy = self.centre
        f, h = self.focal, self.height
        return (cx - f * (dy - self.offset_y) / h,
                cy - f * (dx - self.offset_x) / h)

    def pixel_to_ground(self, u: float, v: float) -> tuple[float, float]:
        """(u, v) -> ground offset from the SENSOR REFERENCE POINT.

        Exact, not approximate: the ground is a known plane at a known
        distance, so there is no scale ambiguity to resolve and no depth
        sensor needed. This is why a nadir camera is worth 40 lines and a
        forward one would be worth a calibration campaign.
        """
        cx, cy = self.centre
        f, h = self.focal, self.height
        return (self.offset_x - (v - cy) * h / f,
                self.offset_y - (u - cx) * h / f)

    def footprint(self) -> tuple[float, float]:
        """(length in x, width in y) of the floor patch it can see."""
        far_x, far_y = self.pixel_to_ground(0.0, 0.0)
        near_x, near_y = self.pixel_to_ground(self.width - 1, self.hpix - 1)
        return abs(far_x - near_x), abs(far_y - near_y)


@dataclass(frozen=True)
class CrossSighting:
    """Where the cross is, relative to the sensor point, in metres."""

    dx: float                 # forward: positive means the cross is ahead
    dy: float                 # left: positive means the cross is to the left
    yaw: float                # cross orientation relative to the robot, +-45 deg
    pixels: int
    fill: float

    @property
    def range(self) -> float:
        return math.hypot(self.dx, self.dy)


def red_mask(rgb: np.ndarray) -> np.ndarray:
    """Boolean mask of the strongly-red pixels."""
    a = rgb.astype(np.int16)
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    return (r >= RED_MIN) & (r > RED_DOMINANCE * g) & (r > RED_DOMINANCE * b)


def _label(mask: np.ndarray) -> tuple[np.ndarray, int]:
    """Connected components, 4-connected, in numpy alone.

    Written out rather than pulled from OpenCV or scipy because this module
    is imported by the robot node, and the robot node must start on a machine
    that has ROS and may not have either. It is a two-pass union-find over
    row runs, which is fast enough on 320x240 and, more to the point, has no
    dependencies to be missing at 9 a.m. on the day of the demonstration.
    """
    h, w = mask.shape
    labels = np.zeros((h, w), dtype=np.int32)
    parent: list[int] = [0]

    def find(a: int) -> int:
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a: int, b: int) -> None:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[max(ra, rb)] = min(ra, rb)

    for y in range(h):
        row, prev = mask[y], labels[y - 1] if y else None
        x = 0
        while x < w:
            if not row[x]:
                x += 1
                continue
            end = x
            while end < w and row[end]:
                end += 1
            above = prev[x:end] if prev is not None else None
            hits = sorted({int(v) for v in above if v}) if above is not None \
                else []
            if hits:
                lab = hits[0]
                for other in hits[1:]:
                    union(lab, other)
            else:
                lab = len(parent)
                parent.append(lab)
            labels[y, x:end] = lab
            x = end

    if len(parent) == 1:
        return labels, 0
    roots = np.array([find(i) for i in range(len(parent))], dtype=np.int32)
    uniq, packed = np.unique(roots[1:], return_inverse=True)
    remap = np.zeros(len(parent), dtype=np.int32)
    remap[1:] = packed + 1
    return remap[labels], len(uniq)


def cross_yaw(ys: np.ndarray, xs: np.ndarray, cu: float, cv: float) -> float:
    """Orientation of a four-fold symmetric shape, in radians, in (-pi/4, pi/4].

    A cross has no unique principal axis -- its two second moments are equal,
    so PCA returns noise. What it does have is a fourth harmonic: summing
    exp(4*i*phi) over the pixels is zero for a disc and maximal for a cross,
    and its argument is four times the arm angle. That is the whole estimator,
    and it is why the marker is a cross rather than a dot.
    """
    du, dv = xs - cu, cv - ys            # dv: image up = body forward
    r2 = du * du + dv * dv
    phi = np.arctan2(dv, du)
    s = float(np.sum(r2 * np.sin(4 * phi)))
    c = float(np.sum(r2 * np.cos(4 * phi)))
    if abs(s) < 1e-9 and abs(c) < 1e-9:
        return 0.0
    # atan2 gives 4*theta in (-pi, pi]; theta is then in (-pi/4, pi/4].
    # The arms are indistinguishable, so 90 degrees of ambiguity is inherent
    # and harmless: the robot only ever needs to know how far it is FROM
    # square, and it is never more than 45 degrees off.
    return math.atan2(s, c) / 4.0


def find_cross(rgb: np.ndarray, camera: NadirCamera | None = None
               ) -> CrossSighting | None:
    """The nearest red cross under the robot, or None.

    NEAREST to the reference point, not largest. In an inner aisle two
    stations are 0.10 m apart and
    both are in the picture; picking the bigger blob would be a coin toss
    decided by lens distortion, and half the visits would be filed under the
    neighbour's label. The robot is trying to stand on the one it is closest
    to, so proximity to the image centre -- which IS the robot's origin -- is
    the criterion that matches the question.
    """
    cam = camera or NadirCamera()
    mask = red_mask(rgb)
    if not mask.any():
        return None
    labels, n = _label(mask)
    if not n:
        return None

    cu, cv = cam.centre
    best: CrossSighting | None = None
    for lab in range(1, n + 1):
        ys, xs = np.nonzero(labels == lab)
        if ys.size < MIN_PIXELS:
            continue
        if (ys.min() == 0 or xs.min() == 0
                or ys.max() == rgb.shape[0] - 1 or xs.max() == rgb.shape[1] - 1):
            # Clipped by the frame edge. Its centroid is pulled inwards by
            # however much is missing, so it would report the cross as
            # NEARER than it is and the robot would stop short. A sighting
            # that cannot be trusted is worth less than no sighting: the
            # caller falls back on odometry, which is not biased.
            continue
        box = (ys.max() - ys.min() + 1) * (xs.max() - xs.min() + 1)
        fill = ys.size / float(box)
        if not FILL_MIN <= fill <= FILL_MAX:
            continue                      # a solid blob is not a cross
        u, v = float(xs.mean()), float(ys.mean())
        dx, dy = cam.pixel_to_ground(u, v)
        s = CrossSighting(dx=dx, dy=dy, yaw=cross_yaw(ys, xs, u, v),
                          pixels=int(ys.size), fill=fill)
        if best is None or s.range < best.range:
            best = s
    return best


# --------------------------------------------------------------- rendering
FLOOR_RGB = (38, 64, 54)          # the aisle paint, as the world defines it
CROSS_RGB = (232, 26, 26)


def render_cross(dx: float, dy: float, yaw: float = 0.0,
                 camera: NadirCamera | None = None,
                 others: tuple[tuple[float, float], ...] = ()) -> np.ndarray:
    """A synthetic frame with a cross at body offset (dx, dy).

    Rendered by back-projecting every pixel to the ground and asking whether
    that point is on a cross arm -- i.e. through pixel_to_ground, the same
    function the detector uses forwards. That makes the round trip a real
    test of the geometry: if a sign is wrong in the projection the detector
    still recovers the offset, but the OFFSET IT RECOVERS is wrong in the
    same direction, so the test compares against the requested dx and dy,
    which never went through the camera at all.
    """
    cam = camera or NadirCamera()
    img = np.zeros((cam.hpix, cam.width, 3), dtype=np.uint8)
    img[:, :] = FLOOR_RGB
    vs, us = np.mgrid[0:cam.hpix, 0:cam.width]
    cx, cy = cam.centre
    f, h = cam.focal, cam.height
    gx = cam.offset_x - (vs - cy) * h / f
    gy = cam.offset_y - (us - cx) * h / f

    for ox, oy in ((dx, dy), *others):
        px, py = gx - ox, gy - oy
        c, s = math.cos(-yaw), math.sin(-yaw)
        ax, ay = c * px - s * py, s * px + c * py
        arm, th = CROSS_ARM, CROSS_THICKNESS / 2.0
        on = (((np.abs(ax) <= arm) & (np.abs(ay) <= th))
              | ((np.abs(ay) <= arm) & (np.abs(ax) <= th)))
        img[on] = CROSS_RGB
    return img
