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
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
# ALIASED on purpose. `OccupancyGrid` is also the name of our own numpy
# log-odds grid, imported below -- and that import comes later in the file, so
# it silently rebinds the name and create_publisher(OccupancyGrid, ...) hands
# rclpy a plain Python class. The error it raises then blames the ROS 1/ROS 2
# message mix-up, which is nowhere near the truth and killed the node on
# startup. mapping_node aliases it for exactly this reason; so does this one.
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
        # Low-gain fusion (complementary-filter style): apply only this
        # fraction of each match correction. Odometry drifts slowly (a few
        # mm per matched step), so 35 % per match still tracks it easily,
        # while any residual systematic matcher bias (e.g. corridor pull)
        # is divided by ~3 instead of integrating at full strength.
        self.declare_parameter("correction_gain", 0.35)
        self.declare_parameter("metrics_period", 5.0)
        # Divergence alarm. Every match is bounded to a +/-0.11 m search
        # window, so SLAM can never JUMP away from the prior -- it can only
        # walk away, one small biased correction at a time, which is exactly
        # what a 36 min run did (1.20 m off truth against a 0.20 m prior).
        # The sum of all applied corrections IS the disagreement with dead
        # reckoning; past this much, the odometry is the more credible of the
        # two and the log should say so instead of leaving it to be
        # reverse-engineered afterwards. Split along/across the heading,
        # because a matcher that lags shows up almost purely along it.
        self.declare_parameter("max_drift_warn", 0.60)   # m
        # MATURITY. Only cells whose evidence is established may vote in the
        # match. This is the fix for the runaway lag, and the mechanism is
        # sharper than "coverage": a cell grazed ONCE by a passing beam gets
        # L_FREE = -0.40, and |−0.40| already passed the old 0.2 gate, so it
        # counted as "known". The frontier just ahead of the robot is full of
        # such cells -- known, but not yet resolved as obstacles. A beam
        # landing there is scored as a MISS, not as unexplored. Advancing was
        # therefore actively punished, and switching the score to a mean made
        # it far worse: under a sum those zeros merely failed to add, under a
        # mean each one drags the average down. Hence 2.03 m of drift in
        # 13 min against 1.20 m in 36 min before. A cell must now be seen
        # about five times (5 x 0.40) or hit three times (3 x 0.85) before its
        # opinion counts; everything fresher is unexplored, which is the truth.
        self.declare_parameter("mature_log_odds", 2.0)
        # ANISOTROPIC GAIN. Down a straight aisle, position ALONG the aisle is
        # unobservable -- sliding forward does not change what the lidar sees
        # (the aperture problem), so any "better" pose found along it is noise
        # or residual bias. Across the aisle and in heading, the plant rows
        # give real information. A proper EKF gets this for free from its
        # covariance; with a fixed-gain filter it has to be said explicitly.
        # Full gain across and in heading, a fraction of it along.
        self.declare_parameter("along_gain_scale", 0.15)

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
        self._gain = float(self.get_parameter("correction_gain").value)
        self._max_drift = float(self.get_parameter("max_drift_warn").value)
        self._mature = float(self.get_parameter("mature_log_odds").value)
        self._along_scale = float(self.get_parameter("along_gain_scale").value)
        self._moved = 0.0          # travel accumulated since the last match
        self._turned = 0.0
        self._corr_along = 0.0     # sum of applied corrections, body frame
        self._corr_cross = 0.0

        self._odom = None          # latest drifting odometry (x, y, yaw)
        self._odom_hist = deque(maxlen=200)   # (t, x, y, yaw) for time pairing
        self._odom_at_scan = None  # drifting odometry at the PREVIOUS scan time
        self._pose = None          # corrected pose (x, y, yaw)
        self._gt = None            # ground truth, metrics only
        self._err_sum = 0.0
        self._err_n = 0

        self._br = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(Odometry, "pose_slam", 20)
        # The map the matcher is actually using, uninflated. mapping_node's
        # /map is a PLANNING grid: it is inflated by the robot radius, so
        # measuring the greenhouse off it would report walls 0.36 m too thick.
        # This one is the raw belief -- what map_eval scores, and what RViz
        # should show if you want to see what the robot really thinks.
        self.map_pub = self.create_publisher(OccupancyGridMsg, "map_slam", 1)
        self.create_timer(2.0, self._publish_map)
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
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._odom = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        self._odom_hist.append((t, *self._odom))
        if self._pose is None:
            # Boot: adopt the odometry pose until the first scan arrives.
            self._pose = self._odom
            self._odom_at_scan = self._odom

    def _odom_at(self, t: float):
        """Odometry sample nearest to time t (the scan's own timestamp).
        Pairing the scan with the LATEST odometry instead skewed the prior by
        however far the robot moved while scans waited in the queue -- with
        the slow field rebuild that reached ~0.3 m and locked the pose."""
        if not self._odom_hist:
            return self._odom
        return min(self._odom_hist, key=lambda s: abs(s[0] - t))[1:]

    def _on_truth(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._gt = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    # ------------------------------------------------------------------- SLAM
    def _on_scan(self, msg: LaserScan) -> None:
        if self._odom is None or self._pose is None:
            return

        # Stale-scan guard: if this scan waited in the queue while the robot
        # kept driving, matching it against a later pose only corrupts things.
        t_scan = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._odom_hist and self._odom_hist[-1][0] - t_scan > 0.4:
            return

        # 1. PREDICT -- odometry increment since the last scan, applied in the
        # body frame of the last corrected pose (standard motion composition).
        # Both endpoints are time-matched to the scans' own timestamps.
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

        # 2. CORRECT -- tight coarse+fine search around the prior, applied at
        # reduced gain (see correction_gain above).
        ranges = list(msg.ranges)
        if self._matcher is not None:
            est, quality = self._matcher.correct_online(
                prior, ranges, msg.range_max)
            g = self._gain
            dth_c = math.atan2(math.sin(est[2] - prior[2]),
                               math.cos(est[2] - prior[2]))
            # Resolve the proposed correction along and across the heading and
            # damp the along-track part (see along_gain_scale), then rebuild
            # the world-frame correction from the two components.
            ch, sh = math.cos(prior[2]), math.sin(prior[2])
            ex, ey = est[0] - prior[0], est[1] - prior[1]
            along = (ch * ex + sh * ey) * self._along_scale
            cross = -sh * ex + ch * ey
            cx, cy = along * ch - cross * sh, along * sh + cross * ch
            self._pose = (prior[0] + g * cx,
                          prior[1] + g * cy,
                          math.atan2(math.sin(prior[2] + g * dth_c),
                                     math.cos(prior[2] + g * dth_c)))
            # Book-keeping only: where the corrections are pushing us,
            # resolved along and across the heading (see max_drift_warn).
            self._corr_along += g * along
            self._corr_cross += g * cross
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
                # Only MATURE cells vote (see mature_log_odds). The distance
                # field stays generous -- one hit is enough to seed a surface,
                # because a missing obstacle would create false misses -- but
                # a cell only gets to say "there is nothing here" once several
                # beams agree with it.
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

    def _publish_map(self) -> None:
        """The log-odds belief as a ROS grid: -1 unknown, 0 free, 100 occupied.

        Anything the robot has no opinion about must stay UNKNOWN rather than
        be published as free -- a map that claims to have seen everything
        scores 100 % coverage without having driven anywhere."""
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
        # Row 0 of a ROS grid is the bottom row; ours is the top one.
        msg.data = data[::-1, :].flatten().tolist()
        self.map_pub.publish(msg)

    def _report(self) -> None:
        if self._err_n == 0 or self._gt is None or self._odom is None:
            return
        slam_err = self._err_sum / self._err_n
        odom_err = math.hypot(self._odom[0] - self._gt[0],
                              self._odom[1] - self._gt[1])
        self._err_sum, self._err_n = 0.0, 0
        # Say WHICH odometry. This node reads /odom_calibrated (the launch
        # remaps it onto odom_noisy): the prior it was actually given, not the
        # raw drifting encoder track truth_monitor prints under the same
        # words. With the calibrator in `calibrate` mode that prior is
        # ground-truth-assisted, and two log lines a second apart then quote
        # 0.07 m and 1.05 m for "odometry alone" -- which is how a whole run
        # gets misread.
        self.get_logger().info(
            f"pose error vs truth: SLAM {slam_err:.02f} m | "
            f"the prior it was given {odom_err:.02f} m")

        drift = math.hypot(self._corr_along, self._corr_cross)
        if drift > self._max_drift:
            self.get_logger().warn(
                f"scan matching has walked {drift:.02f} m away from dead "
                f"reckoning ({self._corr_along:+.02f} m along the heading, "
                f"{self._corr_cross:+.02f} m across). A negative along-track "
                f"figure means the estimate is LAGGING the robot: goals get "
                f"driven at after they have already been passed.",
                throttle_duration_sec=30.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass      # RuntimeError: message mid-shutdown (rclpy)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
