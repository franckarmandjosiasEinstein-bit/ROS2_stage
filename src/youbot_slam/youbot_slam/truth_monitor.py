"""truth_monitor -- side-by-side ground truth vs what the robot believes.

Debugging a stack where localisation, perception and manipulation all feed
each other is guesswork unless you can SEE where each one is wrong. This node
puts reality and belief on the same RViz picture and prints one scoreboard:

    BASE      truth pose vs /pose_slam (and vs raw odometry) -> is it lost?
    WHEELS    each wheel's true ground contact vs where TF puts it
    ARM       true gripper-tip position (from TF) vs the nearest real berry
              -> the direct answer to "is it picking thin air?"
    BERRIES   the generated ground-truth catalogue vs what vision reports,
              split into: in the camera's field of view, detected, missed.

Everything is published as markers on /truth_markers, colour-coded:
    GREEN  = ground truth (reality)
    ORANGE = the robot's estimate
    RED    = an error link between the two, labelled in metres.

Reads worlds/berries.yaml, written by scripts/make_world.py alongside the
world itself, so the reference can never drift out of sync with the scene.

Subscribes:  /odom (truth), /pose_slam, /odom_noisy, /ripe_count, TF
Publishes:   /truth_markers (visualization_msgs/MarkerArray)
"""

from __future__ import annotations

import math
import os

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener

# Robot footprint (m) and wheel mounting points, from youbot_gz.urdf.
BASE_L, BASE_W = 0.58, 0.38
WHEELS = (("wheel_fl", 0.225, 0.19), ("wheel_fr", 0.225, -0.19),
          ("wheel_rl", -0.225, 0.19), ("wheel_rr", -0.225, -0.19))
# Camera, from youbot_gz.urdf: <pose>0.18 0.14 0.78  0 -0.28 1.5708</pose> on
# base_link. The yaw puts the optical axis on the robot's +Y side (the plant
# row it harvests) and the pitch tips it 0.28 rad UP, at the fruit hanging near
# z = 0.9. 640x480 at 1.2 rad horizontal gives 0.95 rad vertically.
CAM_FOV = 1.2
CAM_VFOV = 2.0 * math.atan(math.tan(CAM_FOV / 2.0) * 480.0 / 640.0)
CAM_RANGE = 2.2
CAM_MOUNT = (0.18, 0.14, 0.78)      # in base_link
CAM_PITCH = 0.28                    # rad, upwards
# A berry has to be big enough on the sensor to survive min_pixels. At 640 px
# across 1.2 rad, fx = 468 px, so a 0.032 m berry spans 30/d pixels: a blob of
# ~6 px needs roughly 3 px across, i.e. d < 10 m. Range, not size, is the real
# limit here -- but the check belongs in the model, not in a comment, because
# the moment the camera resolution changes it stops being true.
CAM_FX = (640.0 / 2.0) / math.tan(CAM_FOV / 2.0)
MIN_BLOB_PX = 2.5                   # apparent diameter to be worth reporting


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TruthMonitor(Node):
    def __init__(self) -> None:
        super().__init__("truth_monitor")
        self.declare_parameter("report_period", 10.0)
        self.declare_parameter("berry_file", "")

        self._truth = None      # ground-truth base pose (x, y, yaw)
        self._slam = None       # SLAM estimate
        self._odom = None       # raw drifting odometry
        self._ripe = 0          # clusters vision reports this frame
        self._berries = self._load_catalogue("berries")
        self._foliage = self._load_catalogue("foliage")
        self._closest = None     # best gripper-to-berry distance this window

        self._buf = Buffer()
        self._tf = TransformListener(self._buf, self)
        self.pub = self.create_publisher(MarkerArray, "truth_markers", 5)
        self.create_subscription(Odometry, "odom", self._on_truth, 20)
        self.create_subscription(Odometry, "pose_slam", self._on_slam, 20)
        self.create_subscription(Odometry, "odom_noisy", self._on_odom, 20)
        self.create_subscription(Int32, "ripe_count", self._on_ripe, 10)
        self.create_timer(0.5, self._publish)
        self.create_timer(float(self.get_parameter("report_period").value),
                          self._report)
        self.get_logger().info(
            f"truth_monitor up: {len(self._berries)} reference berries -> "
            "/truth_markers (green = reality, orange = belief)")

    # -------------------------------------------------------------- inputs
    def _load_catalogue(self, section: str):
        """Read one `section:` of berries.yaml as a list of (x, y, z, r)."""
        path = str(self.get_parameter("berry_file").value)
        if not path:
            try:
                path = os.path.join(get_package_share_directory("youbot_gazebo"),
                                    "worlds", "berries.yaml")
            except Exception:
                return []
        out, inside = [], False
        try:
            with open(path) as f:
                for line in f:
                    line = line.strip()
                    if line.endswith(":") and not line.startswith("#"):
                        inside = line[:-1] == section
                        continue
                    if inside and line.startswith("- ["):
                        nums = line[line.index("[") + 1:line.index("]")].split(",")
                        out.append(tuple(float(v) for v in nums))
        except (OSError, ValueError) as exc:
            self.get_logger().warn(f"No {section} reference ({exc}).")
        return out

    def _on_truth(self, msg):
        p = msg.pose.pose
        self._truth = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def _on_slam(self, msg):
        p = msg.pose.pose
        self._slam = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def _on_odom(self, msg):
        p = msg.pose.pose
        self._odom = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def _on_ripe(self, msg):
        self._ripe = int(msg.data)

    # -------------------------------------------------------------- helpers
    def _gripper_tip(self):
        """True gripper position, from the TF chain (forward kinematics)."""
        try:
            t = self._buf.lookup_transform("map", "gripper_base", rclpy.time.Time())
        except Exception:
            return None
        return (t.transform.translation.x, t.transform.translation.y,
                t.transform.translation.z)

    def _camera(self):
        """Camera position and optical axis in world coordinates."""
        x, y, yaw = self._truth
        c, s = math.cos(yaw), math.sin(yaw)
        pos = (x + CAM_MOUNT[0] * c - CAM_MOUNT[1] * s,
               y + CAM_MOUNT[0] * s + CAM_MOUNT[1] * c,
               CAM_MOUNT[2])
        a = yaw + math.pi / 2.0            # the axis looks over the +Y flank
        axis = (math.cos(a) * math.cos(CAM_PITCH),
                math.sin(a) * math.cos(CAM_PITCH),
                math.sin(CAM_PITCH))
        return pos, axis

    def _occluded(self, cam, berry) -> bool:
        """True when a leaf sits on the line of sight to this berry.

        Segment-sphere test: a leaf blocks the view if the segment from the
        camera to the berry passes within the leaf's radius of its centre, and
        does so BEFORE reaching the berry. The berry's own leaf cluster counts
        -- that is the point. Roughly half the fruit on a strawberry plant
        hangs behind its own foliage, and no camera reports those."""
        cx, cy, cz = cam
        bx, by, bz, br = berry[0], berry[1], berry[2], berry[3]
        vx, vy, vz = bx - cx, by - cy, bz - cz
        seg2 = vx * vx + vy * vy + vz * vz
        if seg2 < 1e-9:
            return False
        # Leaves AND other berries. A berry two metres down the row is behind
        # every plant in between -- and behind their fruit. Leaving the fruit
        # out of the occluder set is why the monitor still read "13 visible,
        # vision reports 1" whenever it was looking along a row: the misses
        # were all at 1.5 to 2.2 m, the matches all under 0.6 m.
        for ox, oy, oz, orad in self._foliage + self._berries:
            wx, wy, wz = ox - cx, oy - cy, oz - cz
            t = (wx * vx + wy * vy + wz * vz) / seg2
            # Stop short of the berry itself: a leaf level with the fruit is
            # what it hangs among, not what hides it, and the target must not
            # occlude itself.
            if t <= 0.02 or t >= 0.92:
                continue
            px, py, pz = cx + t * vx, cy + t * vy, cz + t * vz
            if (ox - px) ** 2 + (oy - py) ** 2 + (oz - pz) ** 2 < (orad + br) ** 2:
                return True
        return False

    def _berries_in_view(self):
        """True berries the camera can ACTUALLY see: inside the frustum
        (horizontal and vertical), within range, big enough on the sensor to
        make a blob, and not hidden behind a leaf.

        The first version of this only tested range and horizontal angle, and
        so counted fruit behind foliage, above the frame, and 2 m down the row
        at a grazing angle. Every run then printed "15 in view, vision reports
        2 <-- MISSING", which reads as a perception failure and is not one. A
        reference that overstates what is visible is worse than no reference:
        it sends you tuning a detector that was never the problem."""
        if self._truth is None:
            return []
        cam, axis = self._camera()
        seen = []
        for b in self._berries:
            bx, by, bz, br = b
            vx, vy, vz = bx - cam[0], by - cam[1], bz - cam[2]
            d = math.sqrt(vx * vx + vy * vy + vz * vz)
            if d > CAM_RANGE or d < 1e-6:
                continue
            # Horizontal and vertical angle off the optical axis, separately:
            # a frustum is a rectangle, not a cone.
            ah = math.atan2(vy, vx) - math.atan2(axis[1], axis[0])
            ah = math.atan2(math.sin(ah), math.cos(ah))
            av = math.asin(max(-1.0, min(1.0, vz / d))) - CAM_PITCH
            if abs(ah) > CAM_FOV / 2.0 or abs(av) > CAM_VFOV / 2.0:
                continue
            if 2.0 * br * CAM_FX / d < MIN_BLOB_PX:
                continue
            if self._occluded(cam, b):
                continue
            seen.append((bx, by, bz, br, d))
        return seen

    # -------------------------------------------------------------- markers
    def _marker(self, mid, mtype, scale, colour, frame="map"):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "truth"
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.scale.x, m.scale.y, m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = colour
        m.pose.orientation.w = 1.0
        return m

    def _footprint(self, mid, pose, colour):
        """Robot outline as a line loop at the given pose."""
        m = self._marker(mid, Marker.LINE_STRIP, (0.03, 0.0, 0.0), colour)
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        hl, hw = BASE_L / 2.0, BASE_W / 2.0
        for lx, ly in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw), (hl, hw),
                       (hl * 1.4, 0.0), (hl, -hw)):     # nose spike = heading
            m.points.append(Point(x=x + c * lx - s * ly, y=y + s * lx + c * ly, z=0.03))
        return m

    def _publish(self) -> None:
        if self._truth is None:
            return
        arr = MarkerArray()
        GREEN, ORANGE, RED = (0.1, 0.9, 0.2, 0.9), (1.0, 0.55, 0.1, 0.9), (1.0, 0.1, 0.1, 0.9)

        arr.markers.append(self._footprint(0, self._truth, GREEN))
        if self._slam is not None:
            arr.markers.append(self._footprint(1, self._slam, ORANGE))
            link = self._marker(2, Marker.LINE_LIST, (0.02, 0.0, 0.0), RED)
            link.points = [Point(x=self._truth[0], y=self._truth[1], z=0.03),
                           Point(x=self._slam[0], y=self._slam[1], z=0.03)]
            arr.markers.append(link)
            err = math.hypot(self._slam[0] - self._truth[0], self._slam[1] - self._truth[1])
            label = self._marker(3, Marker.TEXT_VIEW_FACING, (0.0, 0.0, 0.16), RED)
            label.pose.position.x, label.pose.position.y = self._truth[0], self._truth[1]
            label.pose.position.z = 0.75
            label.text = f"pose err {err:.02f} m"
            arr.markers.append(label)

        # Wheels: true mounting points (green) vs where TF places them (orange).
        for i, (name, lx, ly) in enumerate(WHEELS):
            x, y, yaw = self._truth
            c, s = math.cos(yaw), math.sin(yaw)
            dot = self._marker(10 + i, Marker.SPHERE, (0.07, 0.07, 0.07), GREEN)
            dot.pose.position.x = x + c * lx - s * ly
            dot.pose.position.y = y + s * lx + c * ly
            dot.pose.position.z = 0.054
            arr.markers.append(dot)
            try:
                t = self._buf.lookup_transform("map", name, rclpy.time.Time())
                est = self._marker(20 + i, Marker.SPHERE, (0.05, 0.05, 0.05), ORANGE)
                est.pose.position.x = t.transform.translation.x
                est.pose.position.y = t.transform.translation.y
                est.pose.position.z = t.transform.translation.z
                arr.markers.append(est)
            except Exception:
                pass

        # Every reference berry: green, brighter and bigger when in view.
        in_view = {(round(b[0], 4), round(b[1], 4)) for b in self._berries_in_view()}
        for i, (bx, by, bz, br) in enumerate(self._berries):
            live = (round(bx, 4), round(by, 4)) in in_view
            col = (0.2, 1.0, 0.3, 0.95) if live else (0.15, 0.55, 0.2, 0.35)
            m = self._marker(100 + i, Marker.SPHERE,
                             (br * 2.4, br * 2.4, br * 2.4) if live else (br * 1.6,) * 3,
                             col)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = bx, by, bz
            arr.markers.append(m)

        # Arm: gripper tip and its distance to the nearest real berry.
        tip = self._gripper_tip()
        if tip is not None:
            m = self._marker(4, Marker.SPHERE, (0.07, 0.07, 0.07), ORANGE)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = tip
            arr.markers.append(m)
            near = self._nearest_berry(tip)
            if near is not None:
                d, b = near
                if self._closest is None or d < self._closest:
                    self._closest = d
                reach = self._marker(5, Marker.LINE_LIST, (0.015, 0.0, 0.0),
                                     GREEN if d < 0.10 else RED)
                reach.points = [Point(x=tip[0], y=tip[1], z=tip[2]),
                                Point(x=b[0], y=b[1], z=b[2])]
                arr.markers.append(reach)
                lab = self._marker(6, Marker.TEXT_VIEW_FACING, (0.0, 0.0, 0.10),
                                   GREEN if d < 0.10 else RED)
                lab.pose.position.x, lab.pose.position.y = tip[0], tip[1]
                lab.pose.position.z = tip[2] + 0.12
                lab.text = f"gripper->berry {d:.02f} m"
                arr.markers.append(lab)
        self.pub.publish(arr)

    def _nearest_berry(self, p):
        if not self._berries:
            return None
        best, bd = None, 1e9
        for b in self._berries:
            d = math.sqrt((b[0] - p[0]) ** 2 + (b[1] - p[1]) ** 2 + (b[2] - p[2]) ** 2)
            if d < bd:
                bd, best = d, b
        return bd, best

    # ------------------------------------------------------------ scoreboard
    def _report(self) -> None:
        if self._truth is None:
            return
        # Log the true pose AND heading. Without the heading a report saying
        # "9 visible, vision reports 1" cannot be re-checked offline: which
        # berries were in frame depends entirely on which way the camera was
        # pointing, and reconstructing that from the mission's messages is
        # guesswork.
        lines = ["--- truth vs belief --- (truth pose %.2f, %.2f, %.0f deg)"
                 % (self._truth[0], self._truth[1], math.degrees(self._truth[2]))]
        if self._slam is not None:
            e = math.hypot(self._slam[0] - self._truth[0], self._slam[1] - self._truth[1])
            dh = math.degrees(abs(math.atan2(math.sin(self._slam[2] - self._truth[2]),
                                             math.cos(self._slam[2] - self._truth[2]))))
            lines.append(f"BASE    slam {e:.02f} m / {dh:.0f} deg off truth")
        if self._odom is not None:
            e = math.hypot(self._odom[0] - self._truth[0], self._odom[1] - self._truth[1])
            lines.append(f"        odometry alone {e:.02f} m off truth")
        view = self._berries_in_view()
        n = len(view)
        if n == 0 and self._ripe == 0:
            verdict = ""
        elif self._ripe < n:
            verdict = f"  <-- MISSING {n - self._ripe}"
        elif self._ripe > n:
            verdict = f"  <-- {self._ripe - n} FALSE POSITIVE(S)"
        else:
            verdict = "  <-- matched"
        # "visible" here means: in the frustum, close enough, big enough on the
        # sensor, and no leaf on the line of sight. Anything looser overstates
        # what a camera could report and turns every run into a false alarm.
        lines.append(f"BERRIES {n} visible to the camera | vision reports "
                     f"{self._ripe}{verdict}")
        if n:
            lines.append("        nearest visible at %.2f m, farthest %.2f m"
                         % (min(b[4] for b in view), max(b[4] for b in view)))
        # Report the CLOSEST approach over the window, not the instantaneous
        # distance: the arm is stowed most of the time, so sampling at random
        # would always read 'thin air' even on a perfect pick.
        if self._closest is not None:
            d = self._closest
            verdict = "on target" if d < 0.10 else ("close" if d < 0.25 else "THIN AIR")
            lines.append(f"ARM     closest gripper-to-berry approach {d:.02f} m "
                         f"-> {verdict}")
            self._closest = None
        self.get_logger().info("\n".join(lines))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TruthMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass          # RuntimeError: message arriving mid-shutdown (rclpy)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
