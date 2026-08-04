"""Stage 6 -- autonomous navigation in an EMPTY greenhouse.

Empty means empty. No people, no trolleys, no dog. This is the first stage in
which the robot decides where to go by itself, and the only honest way to find
out what it does wrong is to let it, with nobody in the way.

WHAT COUNTS AS FAILURE, STATED IN ADVANCE
Deciding after the fact what counted as a collision is how commissioning
reports become fiction. Three counters, declared by the operator during the
run, and any of them being non-zero fails the stage:

    contact       the robot touched anything
    intervention  a human had to act to prevent contact (E-stop or a shove)
    abandonment   the robot gave up on a goal or got stuck

    ros2 topic pub -1 /commissioning/event std_msgs/String \\
      "{data: contact}"        # or intervention / abandonment / note:<text>

The node additionally measures, without being told: the minimum lidar
clearance actually reached, the completed round trips, and the mean speed. The
minimum clearance is the number that decides whether the protective distance
computed in stage 0 is being respected in practice or only in theory.
"""

from __future__ import annotations

import math

import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, String

from youbot_commissioning.lib.stage import CommissioningStage, run
from youbot_control.lib.clearance import contact_distance


class Stage6(CommissioningStage):
    STAGE = 6
    SLUG = "navigation"
    TITLE = "Autonomous navigation, empty greenhouse"
    PROCEDURE = """
    BEFORE ARMING
      1. Stages 0-5 have all PASSED. In particular stage 0 produced a
         measured stopping distance; the navigation stop distance must be at
         least that plus a margin. Check it before you arm.
      2. THE GREENHOUSE IS EMPTY OF PEOPLE. One operator, outside the aisles,
         holding the E-stop, watching. A second person at the far end.
      3. Speed is bridled. Do not raise it to "see what happens".

    WHAT WILL HAPPEN
      The normal mission stack drives the robot. This node only WATCHES: it
      does not command. It counts round trips, tracks the closest the robot
      ever came to anything, and records the events you declare.

    DECLARING EVENTS (do it immediately, not from memory afterwards)
      ros2 topic pub -1 /commissioning/event std_msgs/String "{data: contact}"
      ros2 topic pub -1 /commissioning/event std_msgs/String "{data: intervention}"
      ros2 topic pub -1 /commissioning/event std_msgs/String "{data: abandonment}"
      ros2 topic pub -1 /commissioning/event std_msgs/String "{data: 'note:leaf brushed the lidar'}"

    PASS MEANS
      The target number of round trips completed, zero contacts, zero
      interventions, zero abandonments, and the minimum clearance never went
      below the protective distance. Anything else is a finding, and findings
      are why this stage exists.
    """

    def __init__(self):
        super().__init__("stage6_navigation")
        self.declare_parameter("target_trips", 20)
        self.declare_parameter("protective_distance", 0.35)   # m from bumper
        self.declare_parameter("half_length", 0.29)
        self.declare_parameter("half_width", 0.19)
        # The band is no longer a parameter here. It used to be 0.24 m, the
        # same stale constant the guard carried, and a certification tool that
        # measures clearance differently from the guard that enforces it will
        # sign off a number the robot does not actually keep. The geometry now
        # comes from youbot_control.lib.clearance, once, for both.
        self.declare_parameter("corridor_margin", 0.02)
        self.declare_parameter("min_valid_range", 0.30)
        self.declare_parameter("lap_topic", "survey_lap")
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("watch_cmd_topic", "cmd_vel")

        # This stage never drives. Make that structural, not a promise.
        self._vmax = 0.0
        self._wmax = 0.0

        self._events = {"contact": 0, "intervention": 0, "abandonment": 0}
        self._notes: list[str] = []
        self._trips = 0
        self._min_clearance = float("inf")
        self._min_clearance_at = None
        self._pose = None
        self._distance = 0.0
        self._last_xy = None
        self._moving_ticks = 0
        self._ticks = 0
        self._speed_sum = 0.0
        self._t0 = None

        self.create_subscription(LaserScan,
                                 str(self.get_parameter("scan_topic").value),
                                 self._on_scan, 10)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(Int32,
                                 str(self.get_parameter("lap_topic").value),
                                 self._on_lap, 10)
        self.create_subscription(Twist,
                                 str(self.get_parameter("watch_cmd_topic").value),
                                 self._on_cmd, 10)
        self.create_subscription(String, "/commissioning/event",
                                 self._on_event, 10)
        self.create_timer(30.0, self._heartbeat)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # --- passive observation -------------------------------------------
    def _on_event(self, msg: String) -> None:
        text = msg.data.strip()
        if text.startswith("note:"):
            self._notes.append(text[5:].strip())
            self.report.note(f"operator note: {text[5:].strip()}")
            return
        if text in self._events:
            self._events[text] += 1
            where = ("unknown" if self._pose is None
                     else f"({self._pose[0]:+.2f}, {self._pose[1]:+.2f})")
            self.get_logger().warn(
                f"EVENT '{text}' recorded at {where} "
                f"(total {self._events[text]})")
            self.report.note(f"{text} at {where}")
        else:
            self.get_logger().warn(f"unknown event '{text}' ignored")

    def _on_lap(self, msg: Int32) -> None:
        self._trips = int(msg.data)
        self.get_logger().info(f"  round trip {self._trips} completed")

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        if self._last_xy is not None:
            self._distance += math.hypot(p.x - self._last_xy[0],
                                         p.y - self._last_xy[1])
        self._last_xy = (p.x, p.y)
        self._pose = (p.x, p.y)
        if self._t0 is None:
            self._t0 = self._now()

    def _on_cmd(self, msg: Twist) -> None:
        self._ticks += 1
        speed = math.hypot(msg.linear.x, msg.linear.y)
        self._speed_sum += speed
        if speed > 0.02:
            self._moving_ticks += 1

    def _on_scan(self, msg: LaserScan) -> None:
        """Closest approach, measured from the FOOTPRINT EDGE.

        Not the raw range. A range is measured from the lidar at the base
        centre, and reporting that as 'clearance' understates the danger by up
        to 0.29 m -- the length of the nose in front of the sensor.

        The computation is imported, not repeated. This stage is the tool that
        decides whether the robot kept its protective distance on real
        hardware; if it measured clearance with its own copy of the geometry it
        could certify a distance the guard never enforced. `contact_distance`
        is the same function safety_node brakes on.
        """
        hl = float(self.get_parameter("half_length").value)
        bw = float(self.get_parameter("half_width").value)
        margin = float(self.get_parameter("corridor_margin").value)
        rmin = float(self.get_parameter("min_valid_range").value)

        ranges = np.asarray(msg.ranges, dtype=float)
        n = ranges.size
        if n == 0:
            return
        ang = msg.angle_min + np.arange(n) * msg.angle_increment
        ok = np.isfinite(ranges) & (ranges > rmin) & (ranges < msg.range_max)
        if not ok.any():
            return
        px = ranges[ok] * np.cos(ang[ok])
        py = ranges[ok] * np.sin(ang[ok])
        # Forward travel: how far the base could go before touching each
        # return, smallest wins. inf/None means it never touches.
        clearance = float("inf")
        for x, y in zip(px, py):
            d = contact_distance(float(x), float(y), 0.0, hl, bw, margin)
            if d is not None and d < clearance:
                clearance = d
        if clearance == float("inf"):
            return
        if clearance < self._min_clearance:
            self._min_clearance = clearance
            self._min_clearance_at = self._pose
            if clearance < float(self.get_parameter("protective_distance").value):
                self.get_logger().warn(
                    f"clearance {clearance:.3f} m from the bumper -- BELOW the "
                    f"protective distance, at {self._min_clearance_at}")

    def _heartbeat(self) -> None:
        elapsed = 0.0 if self._t0 is None else (self._now() - self._t0) / 60.0
        self.get_logger().info(
            f"[stage6] {elapsed:.1f} min | trips {self._trips}/"
            f"{int(self.get_parameter('target_trips').value)} | "
            f"{self._distance:.1f} m | min clearance "
            f"{self._min_clearance:.3f} m | events {self._events}")
        if self._trips >= int(self.get_parameter("target_trips").value):
            self._conclude()
            self.finish()

    def stop(self) -> None:
        super().stop()
        if self._ticks and not self.report.checks:
            self._conclude()

    def _conclude(self) -> None:
        mean_speed = (self._speed_sum / self._ticks) if self._ticks else None
        self.report.record("round_trips", self._trips)
        self.report.record("distance_m", self._distance)
        self.report.record("mean_commanded_speed", mean_speed)
        self.report.record("min_clearance_m",
                           None if math.isinf(self._min_clearance)
                           else self._min_clearance)
        self.report.record("min_clearance_at", self._min_clearance_at)
        self.report.record("events", dict(self._events))
        self.report.record("operator_notes", self._notes)

        self.report.check("round trips completed", self._trips, ">=",
                          int(self.get_parameter("target_trips").value))
        self.report.check("contacts", self._events["contact"], "==", 0)
        self.report.check("human interventions",
                          self._events["intervention"], "==", 0)
        self.report.check("abandoned goals",
                          self._events["abandonment"], "==", 0)
        self.report.check("minimum clearance",
                          None if math.isinf(self._min_clearance)
                          else self._min_clearance, ">=",
                          float(self.get_parameter("protective_distance").value),
                          "m", "measured from the bumper, not the lidar")


def main(args=None) -> None:
    run(Stage6)


if __name__ == "__main__":
    main()
