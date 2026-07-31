"""slam_node -- homemade online SLAM: localise in the map you are building.

Level-3 autonomy for the greenhouse robot (and the reason the same software
can be dropped into a hospital corridor or a warehouse): the robot no longer
receives its position from outside. It gets a *drifting* odometry prior
(/odom_noisy, as real encoders would give) and recovers its true pose by
correlative scan matching against the occupancy grid it is itself building.

The loop, at every lidar scan:

    1. PREDICT   advance the last corrected pose by the body-frame odometry
                 increment since the previous scan (dead reckoning);
    2. CORRECT   coarse+fine likelihood-field search around that prior
                 (OnlineScanMatcher -- see matcher.py for the three online
                 adaptations of the Webots-validated matcher);
    3. UPDATE    fuse the scan into the log-odds map at the corrected pose,
                 but ONLY when the match quality is high -- integrating at an
                 uncertain pose smears the map and the error feeds back;
    4. every N scans, rebuild the matcher from the evolving map.

Bootstrap: the first scans are integrated at the odometry pose (there is no
map to match against yet); matching starts once the map has enough obstacle
surface, and from then on odometry only seeds the search window.

Validated on the ROS-free greenhouse bench (18 m patrol, 4-8 % injected
drift): mean error 0.06 m / final 0.05 m, versus 0.51 m / 1.16 m for the
drifting odometry alone.

Subscribes:  /odom_noisy (nav_msgs/Odometry)  drifting prior
             /scan       (sensor_msgs/LaserScan)
             /odom       (nav_msgs/Odometry)  ground truth, METRICS ONLY
Publishes:   /pose_slam  (nav_msgs/Odometry, frame map)  corrected pose
             TF map -> base_link
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

from youbot_control.lib.occupancy_grid import OccupancyGrid, L_OCC_THRESHOLD

from youbot_slam.matcher import OnlineScanMatcher


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SlamNode(Node):
    def __init__(self) -> None:
        super().__init__("slam_node")
        # 0.05 m: the coarse 0.10 m grid quantised the walls so hard the
        # matcher had spurious peaks 0.3 m off the true pose on the bench.
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("arena_size", 10.0)
        self.declare_parameter("rebuild_every", 5)    # scans between rebuilds
        self.declare_parameter("min_surface_cells", 30)
        self.declare_parameter("quality_gate", 0.55)  # min quality to map-update
        # Match/update only after real motion (the slam_toolbox
        # minimum_travel_* idea). A 2D lidar streams ~10 Hz even when the
        # robot is parked for a 12 s pick: hundreds of near-tie corrections
        # at a standstill random-walked the pose in the first Gazebo run
        # (0.4 -> 6 m error while odometry barely drifted). Between gated
        # scans the pose advances by odometry composition only.
        self.declare_parameter("min_travel", 0.08)    # m
        self.declare_parameter("min_turn", 0.06)      # rad
        self.declare_parameter("metrics_period", 5.0)

        res = float(self.get_parameter("resolution").value)
        size = float(self.get_parameter("arena_size").value)
        # inflation=0: this map is for matching, not path planning; the
        # matcher seeds its likelihood field from obstacle *surfaces*.
        self.grid = OccupancyGrid(resolution=res, arena_size=size, inflation=0.0)
        self._matcher: OnlineScanMatcher | None = None
        self._scans_since_rebuild = 0
        self._rebuild_every = int(self.get_parameter("rebuild_every").value)
        self._min_surface = int(self.get_parameter("min_surface_cells").value)
        self._quality_gate = float(self.get_parameter("quality_gate").value)
        self._min_travel = float(self.get_parameter("min_travel").value)
        self._min_turn = float(self.get_parameter("min_turn").value)
        self._moved = 0.0          # travel accumulated since the last match
        self._turned = 0.0

        self._odom = None          # latest drifting odometry (x, y, yaw)
        self._odom_at_scan = None  # drifting odometry at the previous scan
        self._pose = None          # corrected pose (x, y, yaw)
        self._gt = None            # ground truth, metrics only
        self._err_sum = 0.0
        self._err_n = 0

        self._br = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(Odometry, "pose_slam", 20)
        self.create_subscription(Odometry, "odom_noisy", self._on_noisy, 20)
        self.create_subscription(LaserScan, "scan", self._on_scan, 10)
        self.create_subscription(Odometry, "odom", self._on_truth, 20)
        self.create_timer(float(self.get_parameter("metrics_period").value),
                          self._report)
        self.get_logger().info(
            "slam_node up: /odom_noisy + /scan -> scan matching -> /pose_slam + TF")

    # ------------------------------------------------------------------ inputs
    def _on_noisy(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._odom = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        if self._pose is None:
            # Boot: adopt the odometry pose until the first scan arrives.
            self._pose = self._odom
            self._odom_at_scan = self._odom

    def _on_truth(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._gt = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    # ------------------------------------------------------------------- SLAM
    def _on_scan(self, msg: LaserScan) -> None:
        if self._odom is None or self._pose is None:
            return

        # 1. PREDICT -- odometry increment since the last scan, applied in the
        # body frame of the last corrected pose (standard motion composition).
        ox, oy, oth = self._odom_at_scan
        nx, ny, nth = self._odom
        c, s = math.cos(-oth), math.sin(-oth)
        dxb = c * (nx - ox) - s * (ny - oy)
        dyb = s * (nx - ox) + c * (ny - oy)
        dth = math.atan2(math.sin(nth - oth), math.cos(nth - oth))
        self._odom_at_scan = self._odom

        x, y, th = self._pose
        c, s = math.cos(th), math.sin(th)
        prior = (x + c * dxb - s * dyb,
                 y + s * dxb + c * dyb,
                 math.atan2(math.sin(th + dth), math.cos(th + dth)))

        # Motion gate: below it, dead-reckon only (publish, no match/update).
        self._moved += math.hypot(dxb, dyb)
        self._turned += abs(dth)
        first_map = self._matcher is None and self._scans_since_rebuild == 0
        if (self._moved < self._min_travel and self._turned < self._min_turn
                and not first_map):
            self._pose = prior
            self._publish(msg.header.stamp)
            return
        self._moved, self._turned = 0.0, 0.0

        # 2. CORRECT -- tight coarse+fine search around the prior.
        ranges = list(msg.ranges)
        if self._matcher is not None:
            self._pose, quality = self._matcher.correct_online(
                prior, ranges, msg.range_max)
        else:
            self._pose, quality = prior, 1.0   # bootstrap: trust odometry

        # 3. UPDATE -- only at a confident pose (map<->pose feedback fix).
        if quality > self._quality_gate or self._matcher is None:
            self.grid.integrate_scan(ranges, *self._pose,
                                     msg.angle_min, msg.angle_increment,
                                     msg.range_max)

        # 4. Periodically rebuild the matcher from the evolving map.
        self._scans_since_rebuild += 1
        if self._matcher is None or self._scans_since_rebuild >= self._rebuild_every:
            occupied = (self.grid.log_odds > L_OCC_THRESHOLD)
            if int(occupied.sum()) >= self._min_surface:
                known = np.abs(self.grid.log_odds) > 0.2
                self._matcher = OnlineScanMatcher(
                    occupied.astype(np.uint8), known,
                    self.grid.resolution, self.grid.arena_size,
                    msg.angle_min, msg.angle_increment)
                self._scans_since_rebuild = 0

        self._publish(msg.header.stamp)

        if self._gt is not None:
            self._err_sum += math.hypot(self._pose[0] - self._gt[0],
                                        self._pose[1] - self._gt[1])
            self._err_n += 1

    # ----------------------------------------------------------------- outputs
    def _publish(self, stamp) -> None:
        x, y, th = self._pose
        half = th / 2.0

        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = "map"
        out.child_frame_id = "base_link"
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.orientation.z = math.sin(half)
        out.pose.pose.orientation.w = math.cos(half)
        self.pose_pub.publish(out)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = math.sin(half)
        t.transform.rotation.w = math.cos(half)
        self._br.sendTransform(t)

    def _report(self) -> None:
        if self._err_n == 0 or self._gt is None or self._odom is None:
            return
        slam_err = self._err_sum / self._err_n
        odom_err = math.hypot(self._odom[0] - self._gt[0],
                              self._odom[1] - self._gt[1])
        self._err_sum, self._err_n = 0.0, 0
        self.get_logger().info(
            f"pose error vs truth: SLAM {slam_err:.02f} m | "
            f"odometry alone {odom_err:.02f} m")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
