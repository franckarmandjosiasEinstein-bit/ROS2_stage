"""berry_view -- one camera model, one visibility test, one set of pixel boxes.

WHY THIS IS A SHARED LIBRARY AND NOT TWO COPIES

Two things need to answer "which berries can the camera see, and where are they
in the image?", and they must never disagree:

  * truth_monitor, which SCORES the detector against reality;
  * dataset_capture, which writes the LABELS the detector is trained on.

If the scorer and the label writer used different visibility rules, a model
would be trained to find one set of berries and marked on another. The gap
would look like a detector defect and would be unfixable by working on the
detector. So there is one implementation, here, and both import it.

This is the same rule that produced lib/clearance.py in youbot_control after
the protective stop and the escape manoeuvre spent a run disagreeing about
what "blocked" meant.

WHAT "VISIBLE" MEANS, PRECISELY

A berry counts as visible when all five hold:

  1. it is within range,
  2. it is inside the frustum horizontally AND vertically -- a frustum is a
     rectangle, not a cone, and treating it as a cone counts fruit that is
     above the top of the frame,
  3. its apparent diameter is at least a few pixels,
  4. no leaf lies on the line of sight,
  5. no OTHER berry lies on the line of sight either.

Point 5 is not pedantry. A berry two metres down the row sits behind every
plant in between and behind their fruit. Leaving fruit out of the occluder set
made the monitor read "13 visible, vision reports 1" whenever it looked along a
row, which reads as a perception failure and is not one.

THE PAN HEAD

The camera bearing is an ARGUMENT, not a constant. It used to be baked in as
yaw + pi/2 because the camera was bolted to the chassis facing left. With a pan
head it changes, it enters the projection in series with the robot yaw, and a
5 deg error at 2 m displaces a berry by 17 cm. Callers pass the MEASURED angle
or they do not call.
"""

from __future__ import annotations

import math


class CameraModel:
    """Pinhole camera on a panning head, mounted on a moving base.

    All angles in radians, all lengths in metres, world frame REP-103.
    """

    def __init__(self, width=640, height=480, hfov=1.2, pitch=0.28,
                 max_range=2.2, min_blob_px=2.5,
                 mount_x=0.18, mount_y=0.0, mount_z=0.78, arm=0.14):
        self.width = float(width)
        self.height = float(height)
        self.hfov = float(hfov)
        # Square pixels: the vertical field follows from the aspect ratio.
        self.vfov = 2.0 * math.atan(math.tan(self.hfov / 2.0)
                                    * self.height / self.width)
        self.fx = (self.width / 2.0) / math.tan(self.hfov / 2.0)
        self.fy = self.fx
        self.cx = (self.width - 1.0) / 2.0
        self.cy = (self.height - 1.0) / 2.0
        self.pitch = float(pitch)
        self.max_range = float(max_range)
        self.min_blob_px = float(min_blob_px)
        # Pan axis in base_link, and how far the lens sits out along the head.
        self.mount = (float(mount_x), float(mount_y), float(mount_z))
        self.arm = float(arm)

    # ---------------------------------------------------------- geometry
    def pose(self, robot_xy_yaw, pan):
        """World position and optical axis of the lens.

        `pan` is the head angle relative to the robot heading. The lens rides
        the head, so its body-frame offset swings with it -- treating that as
        a constant moves every estimate by up to 2*arm.
        """
        x, y, yaw = robot_xy_yaw
        bx = self.mount[0] + self.arm * math.cos(pan)
        by = self.mount[1] + self.arm * math.sin(pan)
        c, s = math.cos(yaw), math.sin(yaw)
        pos = (x + bx * c - by * s,
               y + bx * s + by * c,
               self.mount[2])
        a = yaw + pan
        axis = (math.cos(a) * math.cos(self.pitch),
                math.sin(a) * math.cos(self.pitch),
                math.sin(self.pitch))
        return pos, axis, a

    def angles(self, cam, bearing, point):
        """Horizontal and vertical angle of `point` off the optical axis."""
        vx, vy, vz = (point[0] - cam[0], point[1] - cam[1], point[2] - cam[2])
        d = math.sqrt(vx * vx + vy * vy + vz * vz)
        if d < 1e-9:
            return 0.0, 0.0, 0.0
        ah = math.atan2(vy, vx) - bearing
        ah = math.atan2(math.sin(ah), math.cos(ah))
        av = math.asin(max(-1.0, min(1.0, vz / d))) - self.pitch
        return ah, av, d

    def project(self, cam, bearing, point):
        """Pixel coordinates of a world point, or None if it is behind."""
        ah, av, d = self.angles(cam, bearing, point)
        if d < 1e-9 or abs(ah) >= math.pi / 2.0:
            return None
        # Rectilinear projection: tan of the off-axis angle times the focal
        # length. Using the angle itself is a small-angle approximation that
        # is 6% wrong at the edge of a 1.2 rad frame.
        u = self.cx + self.fx * math.tan(ah)
        v = self.cy - self.fy * math.tan(av)
        return u, v, d

    # -------------------------------------------------------- occlusion
    @staticmethod
    def occluded(cam, berry, occluders) -> bool:
        """True when something sits on the line of sight to this berry.

        Segment-sphere test. The berry must not occlude itself, and a leaf
        level with the fruit is what it hangs among rather than what hides it,
        so the test window stops short of the target.
        """
        cx, cy, cz = cam
        bx, by, bz, br = berry[0], berry[1], berry[2], berry[3]
        vx, vy, vz = bx - cx, by - cy, bz - cz
        seg2 = vx * vx + vy * vy + vz * vz
        if seg2 < 1e-9:
            return False
        for ox, oy, oz, orad in occluders:
            wx, wy, wz = ox - cx, oy - cy, oz - cz
            t = (wx * vx + wy * vy + wz * vz) / seg2
            if t <= 0.02 or t >= 0.92:
                continue
            px, py, pz = cx + t * vx, cy + t * vy, cz + t * vz
            if (ox - px) ** 2 + (oy - py) ** 2 + (oz - pz) ** 2 \
                    < (orad + br) ** 2:
                return True
        return False

    # ---------------------------------------------------------- the API
    def visible(self, robot_xy_yaw, pan, berries, occluders):
        """Berries the camera can actually see.

        Returns a list of dicts, each carrying enough to be either scored or
        written as a training label:
            world  (x, y, z, r)      distance  metres
            uv     (u, v)            pixel centre
            box    (cx, cy, w, h)    YOLO box, NORMALISED to [0, 1]
        """
        cam, _axis, bearing = self.pose(robot_xy_yaw, pan)
        out = []
        for b in berries:
            bx, by, bz, br = b[0], b[1], b[2], b[3]
            ah, av, d = self.angles(cam, bearing, (bx, by, bz))
            if d > self.max_range or d < 1e-6:
                continue
            if abs(ah) > self.hfov / 2.0 or abs(av) > self.vfov / 2.0:
                continue
            apparent = 2.0 * br * self.fx / d
            if apparent < self.min_blob_px:
                continue
            if self.occluded(cam, b, occluders):
                continue
            proj = self.project(cam, bearing, (bx, by, bz))
            if proj is None:
                continue
            u, v, _ = proj
            # A sphere projects to (very nearly) a circle of this diameter.
            w = apparent / self.width
            h = apparent / self.height
            box = (u / self.width, v / self.height, w, h)
            # Clip to the frame: a berry straddling the edge is still a berry,
            # but a box that leaves [0,1] is rejected by every training tool.
            if not (0.0 <= box[0] <= 1.0 and 0.0 <= box[1] <= 1.0):
                continue
            out.append({"world": (bx, by, bz, br), "distance": d,
                        "uv": (u, v), "box": box,
                        "angles": (ah, av)})
        return out
