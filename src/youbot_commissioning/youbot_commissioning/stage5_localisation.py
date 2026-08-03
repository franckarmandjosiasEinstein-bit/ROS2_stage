"""Stage 5 -- does the pose estimate stay bounded for half an hour?

Stage 2 measured how fast odometry alone diverges. This stage measures
whether the corrected estimate STOPS diverging, which is the only property
that matters for a machine expected to work a full shift.

The distinction to keep in mind while reading the result:

    odometry error   grows without bound, monotonically, forever
    localised error  oscillates around a floor set by the sensor and the map

A run whose error is 0.20 m at 10 minutes and 0.21 m at 30 minutes has
passed. A run whose error is 0.08 m at 10 minutes and 0.30 m at 30 minutes
has FAILED even though it is more accurate at the start, because the trend is
what predicts hour three.

MEASURING TRUTH WITHOUT A MOTION-CAPTURE RIG
In simulation, read the truth topic. On hardware, the robot returns
periodically to surveyed floor marks, and the operator declares the true pose
at each visit. Cheap, coarse, and sufficient: a tape measure does not drift.
Even better if the site allows it, place AprilTags at surveyed points and let
the camera do the declaring.
"""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Float32MultiArray

from youbot_commissioning.lib.stage import CommissioningStage, run


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class Stage5(CommissioningStage):
    STAGE = 5
    SLUG = "localisation"
    TITLE = "Bounded localisation error over 30 minutes"
    PROCEDURE = """
    BEFORE ARMING
      1. Stage 4 has PASSED and its map is loaded as the reference map.
      2. Localisation is running (AMCL on the saved map, or the project's own
         estimator) and its initial pose has been set.
      3. Mark 3 or 4 reference crosses on the floor at surveyed positions,
         spread across the greenhouse. Write their world coordinates down.

    WHAT WILL HAPPEN
      Nothing autonomous. Push or drive the robot around normally for the full
      duration. Every few minutes, park it exactly on a reference cross and
      declare the true pose:
        ros2 topic pub -1 /commissioning/truth \\
          std_msgs/Float32MultiArray "{data: [x, y, yaw_deg]}"
      The node records the error at each declaration and fits the trend.

    WHAT IS BEING MEASURED
      error at each fix, the worst error, and the SLOPE of error against time.

    PASS MEANS
      The worst error is under the budget AND the slope is flat. A flat slope
      is the whole point: it is the difference between a robot that works for
      an hour and one that works for a shift.
    """

    def __init__(self):
        super().__init__("stage5_localisation")
        self.declare_parameter("duration", 1800.0)       # s, 30 min
        self.declare_parameter("max_position_error", 0.25)   # m
        self.declare_parameter("max_heading_error_deg", 5.0)
        self.declare_parameter("max_drift_rate", 0.10)   # m per 10 min
        self.declare_parameter("min_fixes", 4)
        self.declare_parameter("use_truth_topic", True)
        self.declare_parameter("truth_topic", "truth_pose")
        self.declare_parameter("estimate_topic", "amcl_pose")

        self._est = None
        self._truth = None
        self._t0 = None
        self._fixes: list[dict] = []

        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("estimate_topic").value),
            self._on_estimate, 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter("truth_topic").value),
            self._on_truth_topic, 10)
        self.create_subscription(Float32MultiArray, "/commissioning/truth",
                                 self._on_truth_declared, 10)
        self.create_timer(30.0, self._heartbeat)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_estimate(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        self._est = (p.position.x, p.position.y, _yaw(p.orientation))
        if self._t0 is None:
            self._t0 = self._now()

    def _on_truth_topic(self, msg: PoseStamped) -> None:
        if not bool(self.get_parameter("use_truth_topic").value):
            return
        self._truth = (msg.pose.position.x, msg.pose.position.y,
                       _yaw(msg.pose.orientation))
        self._record_fix("truth_topic")

    def _on_truth_declared(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 3:
            return
        self._truth = (float(msg.data[0]), float(msg.data[1]),
                       math.radians(float(msg.data[2])))
        self._record_fix("operator")

    def _record_fix(self, source: str) -> None:
        if self._est is None or self._truth is None:
            return
        if self._t0 is None:
            self._t0 = self._now()
        t = self._now() - self._t0
        # In sim the truth topic fires continuously; one fix a minute is
        # enough and keeps the two sources comparable.
        if source == "truth_topic" and self._fixes and \
                t - self._fixes[-1]["t"] < 60.0:
            return
        ex = self._est[0] - self._truth[0]
        ey = self._est[1] - self._truth[1]
        eyaw = _wrap(self._est[2] - self._truth[2])
        fix = {"t": t, "source": source,
               "position_error": math.hypot(ex, ey),
               "heading_error_deg": math.degrees(abs(eyaw)),
               "estimate": self._est, "truth": self._truth}
        self._fixes.append(fix)
        self.get_logger().info(
            f"  fix {len(self._fixes)} at t={t / 60.0:.1f} min: "
            f"position error {fix['position_error']:.3f} m, "
            f"heading error {fix['heading_error_deg']:.2f} deg")

    def _heartbeat(self) -> None:
        if self._t0 is None:
            self.get_logger().info("waiting for a pose estimate...")
            return
        t = self._now() - self._t0
        dur = float(self.get_parameter("duration").value)
        self.get_logger().info(
            f"[stage5] {t / 60.0:.1f} / {dur / 60.0:.0f} min, "
            f"{len(self._fixes)} fixes so far")
        if t >= dur:
            self._conclude()
            self.finish()

    def stop(self) -> None:
        super().stop()
        if self._fixes and not self.report.checks:
            self._conclude()

    @staticmethod
    def _slope(fixes) -> float | None:
        """Least-squares slope of position error against time, in m per 600 s."""
        n = len(fixes)
        if n < 3:
            return None
        mt = sum(f["t"] for f in fixes) / n
        me = sum(f["position_error"] for f in fixes) / n
        num = sum((f["t"] - mt) * (f["position_error"] - me) for f in fixes)
        den = sum((f["t"] - mt) ** 2 for f in fixes)
        if den < 1e-9:
            return None
        return (num / den) * 600.0

    def _conclude(self) -> None:
        self.report.record("fixes", self._fixes)
        pos = [f["position_error"] for f in self._fixes]
        head = [f["heading_error_deg"] for f in self._fixes]
        slope = self._slope(self._fixes)
        span = (self._fixes[-1]["t"] - self._fixes[0]["t"]) / 60.0 \
            if len(self._fixes) > 1 else 0.0

        self.report.record("worst_position_error_m", max(pos) if pos else None)
        self.report.record("drift_rate_m_per_10min", slope)
        self.report.record("observed_span_min", span)

        self.report.check("number of fixes", len(self._fixes), ">=",
                          int(self.get_parameter("min_fixes").value))
        self.report.check("observed span", span, ">=", 25.0, "min",
                          "a 30 min claim needs a 30 min run")
        self.report.check("worst position error", max(pos) if pos else None,
                          "<=",
                          float(self.get_parameter("max_position_error").value),
                          "m")
        self.report.check("worst heading error", max(head) if head else None,
                          "<=",
                          float(self.get_parameter("max_heading_error_deg").value),
                          "deg", "heading error is what corrupts fruit "
                                 "back-projection: 5 deg at 2 m is 17 cm")
        self.report.check("drift rate", abs(slope) if slope is not None else None,
                          "<=", float(self.get_parameter("max_drift_rate").value),
                          "m/10min",
                          "a flat slope is the actual requirement; a low "
                          "starting error with a rising trend is a FAIL")


def main(args=None) -> None:
    run(Stage5)


if __name__ == "__main__":
    main()
