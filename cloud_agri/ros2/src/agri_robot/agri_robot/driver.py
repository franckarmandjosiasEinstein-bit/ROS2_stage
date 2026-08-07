"""driver -- the body: everything that moves, sees, or has a wheel in it.

This is the ONLY file in the project that knows Gazebo exists. It implements
agri.robot.Driver, which is three methods:

    drive_to(station) -> the pose actually reached
    photograph(station) -> (jpeg, size)
    pose() -> where we are

Everything above it -- the mission queue, the QR encoding, the ECC sealing,
the acks -- is the same code the offline demo runs. That is deliberate: the
part that can only be tested with a simulator is kept as small as it can be,
and the part that can be tested anywhere is tested everywhere.

HOW IT DRIVES

The greenhouse is known, static and rectangular, so a planner would be
answering a question that is already answered. The free space is four aisles
running along x, joined by a headland at each end:

    four bands in y, between the walls and the three 0.40 m gutters
    a headland at each end, past the ends of the 8 m gutters

so a route is at most three legs: out to a headland, across to the target
band, in to the station. The robot holds yaw = 0 throughout and strafes
sideways -- it is a mecanum base, turning buys nothing and costs the one
thing that went wrong repeatedly in Phase B, which is a rotation near a
gutter.

The geometry itself lives in agri/aisles.py, deliberately NOT here: it is
the part most worth testing and the part that needs Gazebo least. The test
suite sweeps all 2 256 station pairs and checks that the robot's rectangle
never touches a gutter or a wall on any leg. That is how the first version
of the headland coordinate was caught, which had the chassis strafing
through the middle gutter while the boom was safely past its end.

WHY IT STOPS ON THE CROSS AND NOT ON THE COORDINATE

Odometry in this simulation is ground truth, so driving to the catalogue
coordinate would already land within a millimetre and the red crosses would
be decoration. They are not decoration: the brief asks the robot to position
itself on them, and a robot that ignores its marker has not done that. So
the last centimetres are closed on what the floor camera SEES, with the
odometric position as the fallback and a hard cap on how far vision is
allowed to move the robot (VISUAL_MAX_CORRECTION). The cap is what makes
this safe to run: if the detector is ever wrong -- a reflection, a bad
threshold, a sign error nobody caught -- the worst it can do is nothing.
"""

from __future__ import annotations

import io
import math
import threading
import time

import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from PIL import Image as PILImage
from sensor_msgs.msg import Image, JointState, LaserScan
from std_msgs.msg import Float64

from agri.aisles import brake_clearance, route
from agri.catalogue import SENSOR_OFFSET_X, Station
from agri.vision import NadirCamera, find_cross

# --- control ------------------------------------------------------------
RATE = 20.0                    # Hz
V_MAX = 0.45                   # m/s along an aisle
V_FINE = 0.05                  # m/s inside the last few centimetres
K_POS = 1.2                    # P gain on position
K_YAW = 1.5                    # P gain holding yaw at 0
W_MAX = 0.6                    # rad/s
WAYPOINT_TOL = 0.05            # m, good enough for a via point
DOCK_TOL = 0.015               # m, the last leg
SETTLE_TIME = 0.4              # s of stillness before a pose is believed
LEG_TIMEOUT = 60.0             # s per leg before giving up out loud

# --- vision -------------------------------------------------------------
#: The most the floor camera is ever allowed to move the robot.
#:
#: The bound is not a preference. Two stations in an inner aisle are 0.10 m
#: apart, so a correction of more than 0.05 m could leave the robot nearer
#: the NEIGHBOUR than the station it is about to file the reading under --
#: and the Cloud would then reject the report, correctly, for a reason that
#: has nothing to do with the plant. 0.04 keeps a clear margin under that and
#: is still four times the residual the odometric approach leaves behind.
VISUAL_MAX_CORRECTION = 0.04
VISUAL_TRIES = 4
#: Give up on vision below this and say so, rather than reporting a
#: confidence the picture does not support.
VISUAL_SETTLE = 0.008          # m: close enough, stop nudging

# --- lidar brake --------------------------------------------------------
#: Nothing should ever be in the aisle. If something is, stop -- do not
#: swerve. A swerve near a 0.16 m clearance is how a robot ends up on a
#: gutter, and this project has no reason to be clever about obstacles.
#:
#: The distances are measured from the robot's OUTLINE, not from the lidar,
#: and only inside the corridor it is about to sweep. See
#: agri.aisles.brake_clearance for why both of those matter, and for the two
#: false brakes that paid for them.
BRAKE_RANGE = 0.35             # m of clear floor wanted ahead of the bumper
BRAKE_LOOK = 3.0               # m: beyond this a return cannot be relevant
#: More negative than this and the return is inside the robot: the camera
#: pedestal and the mast both cross the lidar's plane, 0.12 and 0.22 m ahead.
SELF_HIT = -0.02
BRAKE_PATIENCE = 8.0           # s of waiting before the leg is failed


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class DriveError(RuntimeError):
    """A leg that could not be completed. Carries why, for the ack."""


class GazeboDriver:
    """agri.robot.Driver, implemented against the ROS 2 / Gazebo topics."""

    def __init__(self, node, *, sensors, log=None) -> None:
        self.node = node
        self.read_sensors = sensors
        self.log = log or node.get_logger().info
        self.camera = NadirCamera()

        self._lock = threading.Lock()
        self._odom: tuple[float, float, float] | None = None
        self._twist: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._scan: LaserScan | None = None
        self._plant_img: Image | None = None
        self._floor_img: Image | None = None
        self._pan: float | None = None
        self.last_sighting = None
        self.visual_used = 0
        self.visual_missed = 0

        qos = 10
        self._cmd = node.create_publisher(Twist, "cmd_vel", qos)
        self._pan_cmd = node.create_publisher(Float64, "camera_pan_cmd", qos)
        node.create_subscription(Odometry, "odom", self._on_odom, qos)
        node.create_subscription(LaserScan, "scan", self._on_scan, qos)
        node.create_subscription(Image, "camera/image", self._on_plant, qos)
        node.create_subscription(Image, "floor_cam/image", self._on_floor, qos)
        node.create_subscription(JointState, "gz_joint_states",
                                 self._on_joints, qos)

    # ------------------------------------------------------------ inbound
    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        t = msg.twist.twist
        with self._lock:
            self._odom = (msg.pose.pose.position.x,
                          msg.pose.pose.position.y, yaw)
            # Straight from the odometry, in the robot's frame, unfiltered.
            # A smoothed speed would look tidier and would hide the thing
            # this field exists to catch: a reading taken while still rolling.
            self._twist = (t.linear.x, t.linear.y, t.angular.z)

    def _on_scan(self, msg: LaserScan) -> None:
        with self._lock:
            self._scan = msg

    def _on_plant(self, msg: Image) -> None:
        with self._lock:
            self._plant_img = msg

    def _on_floor(self, msg: Image) -> None:
        with self._lock:
            self._floor_img = msg

    def _on_joints(self, msg: JointState) -> None:
        if "j_camera_pan" in msg.name:
            with self._lock:
                self._pan = msg.position[msg.name.index("j_camera_pan")]

    # -------------------------------------------------------------- state
    def base_pose(self) -> tuple[float, float, float]:
        with self._lock:
            if self._odom is None:
                raise DriveError("no odometry yet -- is the bridge running?")
            return self._odom

    def pose(self) -> tuple[float, float, float]:
        """The SENSOR point, which is what a station names. Not base_link."""
        x, y, yaw = self.base_pose()
        return (x + SENSOR_OFFSET_X * math.cos(yaw),
                y + SENSOR_OFFSET_X * math.sin(yaw), yaw)

    def velocity(self) -> tuple[float, float, float]:
        """(vx, vy, wz) in the robot's frame. Travels with every report."""
        with self._lock:
            return self._twist

    def wait_ready(self, timeout: float = 30.0) -> None:
        end = time.time() + timeout
        while time.time() < end:
            with self._lock:
                if self._odom is not None:
                    return
            time.sleep(0.1)
        raise DriveError(f"no /odom after {timeout:.0f} s")

    # ------------------------------------------------------------- motion
    def stop(self) -> None:
        self._cmd.publish(Twist())

    def _blocked(self, wvx: float, wvy: float) -> bool:
        """Something in the corridor the robot is about to sweep.

        The geometry is in agri.aisles.brake_clearance, not here, so it can
        be exercised against the cases that actually matter -- the robot's
        own pedestal, a gutter wall being driven past, a gutter end being
        strafed past -- on a machine with no lidar in it.
        """
        with self._lock:
            scan = self._scan
        speed = math.hypot(wvx, wvy)
        if scan is None or speed < 1e-3:
            return False
        _, _, yaw = self.base_pose()
        heading = _wrap(math.atan2(wvy, wvx) - yaw)      # body frame
        dx, dy = math.cos(heading), math.sin(heading)

        for i, r in enumerate(scan.ranges):
            if not (scan.range_min < r < BRAKE_LOOK):
                continue                                 # nothing, or miles off
            a = scan.angle_min + i * scan.angle_increment
            clear = brake_clearance(r * math.cos(a), r * math.sin(a), dx, dy)
            if clear is None or clear < SELF_HIT:
                continue                     # beside the corridor, or is us
            if clear < BRAKE_RANGE:
                return True
        return False

    def _drive(self, tx: float, ty: float, tol: float, what: str) -> None:
        """Hold yaw at 0 and close on (tx, ty) -- a SENSOR-point target."""
        period = 1.0 / RATE
        deadline = time.time() + LEG_TIMEOUT
        blocked_since: float | None = None
        still_since: float | None = None

        while True:
            x, y, yaw = self.pose()
            ex, ey = tx - x, ty - y
            dist = math.hypot(ex, ey)

            if dist < tol:
                # Stop FIRST, then wait. Continuing to command V_FINE while
                # counting down the settle time drives straight through the
                # target and the next pass has to come back for it.
                self.stop()
                if still_since is None:
                    still_since = time.time()
                elif time.time() - still_since > SETTLE_TIME:
                    return
                time.sleep(period)
                continue
            still_since = None

            if time.time() > deadline:
                self.stop()
                raise DriveError(
                    f"{what}: still {dist:.3f} m short after "
                    f"{LEG_TIMEOUT:.0f} s (wanted {tx:+.2f},{ty:+.2f})")

            speed = min(V_MAX, max(V_FINE, K_POS * dist))
            wvx, wvy = (ex / dist) * speed, (ey / dist) * speed

            if self._blocked(wvx, wvy):
                blocked_since = blocked_since or time.time()
                self.stop()
                if time.time() - blocked_since > BRAKE_PATIENCE:
                    raise DriveError(
                        f"{what}: something is in the aisle {BRAKE_RANGE:.2f} m "
                        "ahead and has not moved")
                time.sleep(period)
                continue
            blocked_since = None

            # World -> body. yaw is held near zero, so this is nearly the
            # identity, but writing it out means a yaw disturbance degrades
            # the motion instead of sending the robot sideways.
            c, s = math.cos(yaw), math.sin(yaw)
            cmd = Twist()
            cmd.linear.x = c * wvx + s * wvy
            cmd.linear.y = -s * wvx + c * wvy
            cmd.angular.z = max(-W_MAX, min(W_MAX, -K_YAW * _wrap(yaw)))
            self._cmd.publish(cmd)
            time.sleep(period)

    # --------------------------------------------------------------- API
    def drive_to(self, s: Station) -> tuple[float, float, float]:
        x, y, _ = self.pose()
        legs = route(x, y, s.x, s.y)
        for i, (wx, wy) in enumerate(legs):
            last = i == len(legs) - 1
            self._drive(wx, wy, DOCK_TOL if last else WAYPOINT_TOL,
                        f"{s.label} leg {i + 1}/{len(legs)}")
        self._centre_on_cross(s)
        return self.pose()

    def _centre_on_cross(self, s: Station) -> None:
        """Close the last centimetres on what the camera sees."""
        for _ in range(VISUAL_TRIES):
            sighting = self.look_down()
            if sighting is None:
                self.visual_missed += 1
                self.log(f"{s.label}: no cross in the floor camera -- "
                         "parked on odometry alone")
                return
            self.last_sighting = sighting
            err = sighting.range
            if err < VISUAL_SETTLE:
                self.visual_used += 1
                return
            if err > VISUAL_MAX_CORRECTION:
                # Refuse, loudly. Either the detector is wrong or the robot
                # is not where it thinks it is; moving 20 cm on the word of
                # a red blob is how a visit gets filed under the neighbour.
                self.log(f"{s.label}: the floor camera says the cross is "
                         f"{err:.3f} m away, more than the "
                         f"{VISUAL_MAX_CORRECTION:.2f} m cap -- ignoring it "
                         "and keeping the odometric position")
                self.visual_missed += 1
                return
            x, y, yaw = self.pose()
            c, sn = math.cos(yaw), math.sin(yaw)
            tx = x + c * sighting.dx - sn * sighting.dy
            ty = y + sn * sighting.dx + c * sighting.dy
            self._drive(tx, ty, VISUAL_SETTLE, f"{s.label} visual centring")
        self.visual_used += 1

    def look_down(self):
        """The current floor-camera sighting, or None."""
        rgb = self._as_array(self._floor_img)
        return None if rgb is None else find_cross(rgb, self.camera)

    def photograph(self, s: Station) -> tuple[bytes, tuple[int, int]]:
        """Point the head at the plant, wait for it to STOP, then shoot."""
        want = s.pan_for(self.pose()[2])
        self._pan_cmd.publish(Float64(data=float(want)))
        end = time.time() + 6.0
        while time.time() < end:
            with self._lock:
                got = self._pan
            if got is not None and abs(_wrap(got - want)) < 0.02:
                break
            time.sleep(0.05)
        else:
            self.log(f"{s.label}: the head did not settle at "
                     f"{math.degrees(want):+.0f} deg; photographing anyway")
        # One more camera period after the head stops: an image published
        # while it was still moving carries an unknown angle and a blur.
        time.sleep(0.35)

        with self._lock:
            msg = self._plant_img
        rgb = self._as_array(msg)
        if rgb is None:
            raise DriveError("no image from the plant camera")
        buf = io.BytesIO()
        PILImage.fromarray(rgb).save(buf, format="JPEG", quality=82)
        return buf.getvalue(), (rgb.shape[1], rgb.shape[0])

    @staticmethod
    def _as_array(msg) -> np.ndarray | None:
        """sensor_msgs/Image -> HxWx3 uint8 RGB, or None."""
        if msg is None:
            return None
        a = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding in ("rgb8", "bgr8"):
            a = a.reshape(msg.height, msg.width, 3)
            return a[:, :, ::-1] if msg.encoding == "bgr8" else a
        if msg.encoding in ("rgba8", "bgra8"):
            a = a.reshape(msg.height, msg.width, 4)[:, :, :3]
            return a[:, :, ::-1] if msg.encoding == "bgra8" else a
        raise DriveError(f"unsupported image encoding {msg.encoding!r}")
