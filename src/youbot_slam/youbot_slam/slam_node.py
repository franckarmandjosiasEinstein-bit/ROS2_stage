"""slam_node -- homemade online SLAM: localise in the map you are building.

Level-3 autonomy for the greenhouse robot: the robot no longer receives its
position from outside. It gets a *drifting* odometry prior (/odom_noisy, as
real encoders would give) and recovers its true pose by correlative scan
matching against the occupancy grid it is itself building.

The loop, at every lidar scan:

    1. PREDICT   advance the last corrected pose by the body-frame odometry
                 increment since the previous scan (dead reckoning);
    2. CORRECT   coarse+fine likelihood-field search around that prior
                 (OnlineScanMatcher), with per-axis curvature-adaptive gains
                 so unobservable axes (along a corridor) are left to odometry;
    3. UPDATE    fuse the scan into the log-odds map at the corrected pose,
                 but ONLY when the match quality is high;
    4. every N scans, rebuild the matcher from the evolving map.

Subscribes:  /odom_noisy (nav_msgs/Odometry)  drifting prior
             /scan       (sensor_msgs/LaserScan)
             /odom       (nav_msgs/Odometry)  ground truth, METRICS ONLY
Publishes:   /pose_slam  (nav_msgs/Odometry, frame map)  corrected pose
             TF map -> base_link
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
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
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("arena_size", 10.0)
        self.declare_parameter("rebuild_every", 5)
        self.declare_parameter("min_surface_cells", 30)
        self.declare_parameter("quality_gate", 0.55)
        self.declare_parameter("min_travel", 0.08)
        self.declare_parameter("min_turn", 0.06)
        self.declare_parameter("correction_gain", 0.40)
        self.declare_parameter("metrics_period", 5.0)
        self.declare_parameter("mature_log_odds", 2.0)

        res = float(self.get_parameter("resolution").value)
        size = float(self.get_parameter("arena_size").value)
        self.grid = OccupancyGrid(resolution=res, arena_size=size, inflation=0.0)
        self._matcher: OnlineScanMatcher | None = None
        self._scans_since_rebuild = 0
        self._rebuild_every = int(self.get_parameter("rebuild_every").value)
        self._min_surface = int(self.get_parameter("min_surface_cells").value)
        self._quality_gate = float(self.get_parameter("quality_gate").value)
        self._min_travel = float(self.get_parameter("min_travel").value)
        self._min_turn = float(self.get_parameter("min_turn").value)
        self._gain = float(self.get_parameter("correction_gain").value)
        self._mature = float(self.get_parameter("mature_log_odds").value)
        self._moved = 0.0
        self._turned = 0.0

        self._odom = None
        self._odom_hist = deque(maxlen=200)
        self._odom_at_scan = None
        self._pose = None
        self._gt = None
        self._err_sum = 0.0
        self._err_n = 0

        self._br = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(Odometry, "pose_slam", 20)
        self.map_pub = self.create_publisher(OccupancyGridMsg, "map_slam", 1)
        self.create_timer(2.0, self._publish_map)
        self.create_subscription(Odometry, "odom_noisy", self._on_noisy, 20)
        self.create_subscription(LaserScan, "scan", self._on_scan, 10)
        self.create_subscription(Odometry, "odom", self._on_truth, 20)
        self.create_timer(float(self.get_parameter("metrics_period").value),
                          self._report)
        self.get_logger().info(
            "slam_node up: /odom_noisy + /scan -> scan matching -> /pose_slam + TF")

    def _on_noisy(self, msg: Odometry) -> None:
        p = msg.pose.pose
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._odom = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        self._odom_hist.append((t, *self._odom))
        if self._pose is None:
            self._pose = self._odom
            self._odom_at_scan = self._odom

    def _odom_at(self, t: float):
        if not self._odom_hist:
            return self._odom
        return min(self._odom_hist, key=lambda s: abs(s[0] - t))[1:]

    def _on_truth(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._gt = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def _on_scan(self, msg: LaserScan) -> None:
        if self._odom is None or self._pose is None:
            return

        t_scan = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._odom_hist and self._odom_hist[-1][0] - t_scan > 0.4:
            return

        # 1. PREDICT
        ox, oy, oth = self._odom_at_scan
        nx, ny, nth = self._odom_at(t_scan)
        c, s = math.cos(-oth), math.sin(-oth)
        dxb = c * (nx - ox) - s * (ny - oy)
        dyb = s * (nx - ox) + c * (ny - oy)
        dth = math.atan2(math.sin(nth - oth), math.cos(nth - oth))
        self._odom_at_scan = (nx, ny, nth)

        x, y, th = self._pose
        c, s = math.cos(th), math.sin(th)
        prior = (x + c * dxb - s * dyb,
                 y + s * dxb + c * dyb,
                 math.atan2(math.sin(th + dth), math.cos(th + dth)))

        self._moved += math.hypot(dxb, dyb)
        self._turned += abs(dth)
        first_map = self._matcher is None and self._scans_since_rebuild == 0
        if (self._moved < self._min_travel and self._turned < self._min_turn
                and not first_map):
            self._pose = prior
            self._publish(msg.header.stamp)
            return
        self._moved, self._turned = 0.0, 0.0

        # 2. CORRECT
        ranges = list(msg.ranges)
        if self._matcher is not None:
            est, quality, axis_gains = self._matcher.correct_online(
                prior, ranges, msg.range_max)
            g = self._gain
            dx = est[0] - prior[0]
            dy = est[1] - prior[1]
            dth_c = math.atan2(math.sin(est[2] - prior[2]),
                               math.cos(est[2] - prior[2]))
            self._pose = (prior[0] + g * dx,
                          prior[1] + g * dy,
                          math.atan2(math.sin(prior[2] + g * dth_c),
                                     math.cos(prior[2] + g * dth_c)))
        else:
            self._pose, quality = prior, 1.0

        # 3. UPDATE
        if quality > self._quality_gate or self._matcher is None:
            self.grid.integrate_scan(ranges, *self._pose,
                                     msg.angle_min, msg.angle_increment,
                                     msg.range_max)

        # 4. Rebuild matcher periodically
        self._scans_since_rebuild += 1
        if self._matcher is None or self._scans_since_rebuild >= self._rebuild_every:
            occupied = (self.grid.log_odds > L_OCC_THRESHOLD)
            if int(occupied.sum()) >= self._min_surface:
                known = np.abs(self.grid.log_odds) > self._mature
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

    def _publish_map(self) -> None:
        lo = self.grid.log_odds
        data = np.full(lo.shape, -1, dtype=np.int8)
        data[lo < -0.2] = 0
        data[lo > L_OCC_THRESHOLD] = 100

        msg = OccupancyGridMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = self.grid.resolution
        msg.info.height, msg.info.width = lo.shape
        msg.info.origin.position.x = -self.grid.arena_size / 2.0
        msg.info.origin.position.y = -self.grid.arena_size / 2.0
        msg.info.origin.orientation.w = 1.0
        msg.data = data[::-1, :].flatten().tolist()
        self.map_pub.publish(msg)

    def _report(self) -> None:
        if self._err_n == 0 or self._gt is None or self._odom is None:
            return
        slam_err = self._err_sum / self._err_n
        odom_err = math.hypot(self._odom[0] - self._gt[0],
                              self._odom[1] - self._gt[1])
        self._err_sum, self._err_n = 0.0, 0
        winner = "SLAM" if slam_err < odom_err else "odometry"
        gains = "(no matcher)"
        if self._matcher is not None:
            gx, gy, gth = self._matcher.last_axis_gains
            gains = f"axis gains X={gx:.2f} Y={gy:.2f} th={gth:.2f}"
        self.get_logger().info(
            f"pose error: SLAM {slam_err:.02f} m | prior {odom_err:.02f} m "
            f"({winner} wins) | {gains}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
