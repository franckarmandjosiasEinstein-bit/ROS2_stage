"""perf_monitor -- how long the robot takes, and where the time goes.

A harvester that maps the greenhouse perfectly and picks two berries an hour is
not a working harvester. Accuracy was already measured (truth_monitor for the
pose and the fruit, map_eval for the map); this is the missing axis.

It answers four questions, all from data the stack already publishes:

  HOW LONG   mapping laps, harvest rounds, total mission time -- in SIM
             seconds, with the real-time factor stated so wall-clock numbers
             can be recovered. Sim time is the honest unit: it is what the
             robot would take in the field, independent of the machine the
             simulation runs on.
  HOW FAST   distance covered and mean speed against the 0.50 m/s cruise
             setting. A mean far below cruise means the robot spends its life
             braking, not driving.
  WHERE IT   every control tick is put in exactly one bucket:
  GOES         driving  -- asked to move, and moving
               BLOCKED  -- asked to move, and not moving (the guard is holding
                           the translation, or the base is jammed)
               working  -- deliberately stopped: aligning on a berry, or the
                           arm is picking
             The BLOCKED share is the number that matters. In the run of
             2026-08-03 it was over a third of the harvest phase: 41 "Stuck at"
             events and two 60 s goal timeouts, all at the two ends of the
             y = +0.6 aisle.
  THROUGHPUT berries per sim-minute, and sim-seconds per berry. This is the
             figure a grower would actually be quoted.

Subscribes: /odom (ground truth), /cmd_vel_raw (what the controllers ask for),
            /harvest_count, /survey_lap
Publishes:  nothing -- it reports to the log, on a timer and once at shutdown.
"""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32


class PerfMonitor(Node):
    def __init__(self) -> None:
        super().__init__("perf_monitor")
        self.declare_parameter("report_period", 120.0)  # s of sim time
        self.declare_parameter("rate", 10.0)            # Hz of bucketing
        self.declare_parameter("survey_laps", 3)
        self.declare_parameter("cruise_speed", 0.50)    # m/s, for reference
        # Asked to move but going slower than this = not moving.
        self.declare_parameter("moving_speed", 0.05)    # m/s
        self.declare_parameter("commanded_speed", 0.03) # m/s

        self._laps_total = int(self.get_parameter("survey_laps").value)
        self._cruise = float(self.get_parameter("cruise_speed").value)
        self._v_move = float(self.get_parameter("moving_speed").value)
        self._v_cmd = float(self.get_parameter("commanded_speed").value)

        self._t0 = None          # sim time of the first pose
        self._wall0 = time.time()
        self._pose = None
        self._speed = 0.0
        self._last_pose_t = None
        self._cmd = 0.0          # magnitude of the commanded translation
        self._dist = 0.0
        self._phase = "survey"
        self._lap_marks = []     # sim time at the end of each mapping lap
        self._harvest_t0 = None  # sim time harvesting started
        self._berries = 0
        self._first_berry_t = None
        self._buckets = {"driving": 0.0, "blocked": 0.0, "working": 0.0}
        self._h_buckets = {"driving": 0.0, "blocked": 0.0, "working": 0.0}

        self.create_subscription(Odometry, "odom", self._on_truth, 20)
        self.create_subscription(Twist, "cmd_vel_raw", self._on_cmd, 20)
        self.create_subscription(Int32, "harvest_count", self._on_harvest, 10)
        self.create_subscription(Int32, "survey_lap", self._on_lap, 10)
        self._dt = 1.0 / float(self.get_parameter("rate").value)
        self.create_timer(self._dt, self._sample)
        self.create_timer(float(self.get_parameter("report_period").value),
                          lambda: self._report("periodic"))
        self.get_logger().info(
            "perf_monitor up: timing, throughput and where the time goes.")

    # ------------------------------------------------------------- inputs
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_truth(self, msg: Odometry) -> None:
        t = self._now()
        p = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self._t0 is None:
            self._t0 = t
        if self._pose is not None and self._last_pose_t is not None:
            d = math.hypot(p[0] - self._pose[0], p[1] - self._pose[1])
            dt = t - self._last_pose_t
            # Ground truth can jump on a reset; a 1 m step in 30 ms is not
            # distance travelled, it is a discontinuity. Drop it rather than
            # inflating the odometer.
            if 0.0 < dt < 1.0 and d < 1.0:
                self._dist += d
                self._speed = d / dt
        self._pose, self._last_pose_t = p, t

    def _on_cmd(self, msg: Twist) -> None:
        self._cmd = math.hypot(msg.linear.x, msg.linear.y)

    def _on_harvest(self, msg: Int32) -> None:
        self._berries = int(msg.data)
        if self._first_berry_t is None:
            self._first_berry_t = self._now()

    def _on_lap(self, msg: Int32) -> None:
        lap = int(msg.data)
        self._lap_marks.append(self._now())
        self._report(f"after mapping lap {lap}")
        if lap >= self._laps_total:
            self._phase = "harvest"
            self._harvest_t0 = self._now()

    # -------------------------------------------------------------- clock
    def _sample(self) -> None:
        """One control tick into exactly one bucket."""
        if self._t0 is None:
            return
        if self._cmd < self._v_cmd:
            k = "working"          # deliberately stopped: aligning or picking
        elif self._speed < self._v_move:
            k = "blocked"          # asked to move and not moving
        else:
            k = "driving"
        self._buckets[k] += self._dt
        if self._phase == "harvest":
            self._h_buckets[k] += self._dt

    # ------------------------------------------------------------- report
    @staticmethod
    def _mmss(s: float) -> str:
        return "%d:%02d" % (int(s) // 60, int(s) % 60)

    def _share(self, buckets) -> str:
        total = sum(buckets.values())
        if total <= 0.0:
            return "no data yet"
        return " | ".join(
            "%s %.0f%% (%s)" % (k, 100.0 * buckets[k] / total,
                                self._mmss(buckets[k]))
            for k in ("driving", "blocked", "working"))

    def _report(self, when: str) -> None:
        if self._t0 is None:
            return
        t = self._now()
        elapsed = t - self._t0
        wall = time.time() - self._wall0
        rtf = elapsed / wall if wall > 1e-6 else 0.0
        lines = [f"--- time & throughput ({when}) ---"]
        lines.append(
            "mission   %s of sim time (%s on the wall, real-time factor %.02f)"
            % (self._mmss(elapsed), self._mmss(wall), rtf))

        if self._lap_marks:
            prev = self._t0
            laps = []
            for i, m in enumerate(self._lap_marks, 1):
                laps.append("lap %d %s" % (i, self._mmss(m - prev)))
                prev = m
            lines.append("mapping   " + " | ".join(laps))
        if self._harvest_t0 is not None:
            lines.append("harvest   %s since the map was finished"
                         % self._mmss(t - self._harvest_t0))

        lines.append(
            "motion    %.01f m covered, mean %.02f m/s over the whole mission "
            "(cruise setting %.02f)"
            % (self._dist, self._dist / elapsed if elapsed > 0 else 0.0,
               self._cruise))
        lines.append("time      " + self._share(self._buckets))
        if self._harvest_t0 is not None:
            lines.append("  harvest " + self._share(self._h_buckets))

        if self._berries > 0 and self._harvest_t0 is not None:
            span = max(1e-6, t - self._harvest_t0)
            lines.append(
                "yield     %d strawberries | %.01f per sim-minute | "
                "%.0f s of sim time each"
                % (self._berries, 60.0 * self._berries / span,
                   span / self._berries))
        else:
            lines.append("yield     none picked yet")
        # The honest headline: the share of harvest time that produced nothing
        # because the base was asked to move and did not.
        h_total = sum(self._h_buckets.values())
        if h_total > 60.0:
            lost = self._h_buckets["blocked"]
            lines.append(
                "verdict   %s of the harvest was spent blocked (%.0f%%) -- "
                "%.0f more strawberries at the current rate if it were driving."
                % (self._mmss(lost), 100.0 * lost / h_total,
                   self._berries * lost / max(1e-6, h_total - lost)))
        self.get_logger().info("\n".join(lines))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerfMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        # The final numbers are the ones worth having; print them even on
        # Ctrl-C, which is how every run of this project actually ends.
        try:
            node._report("final")
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
