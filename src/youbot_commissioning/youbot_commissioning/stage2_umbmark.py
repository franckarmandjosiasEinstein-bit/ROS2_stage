"""Stage 2 -- UMBmark odometry calibration on the REAL floor.

This is the stage that will tell you, earliest and cheapest, how far the real
greenhouse is from the simulation. It costs half a day and it decides whether
the localisation plan is adequate or has to be redesigned.

THE METHOD (Borenstein & Feng, 1996)
Drive a square of side L, clockwise, five times, and measure where the robot
really ends up each time. Then drive the same square counter-clockwise five
times. Two systematic errors dominate wheeled odometry and the two directions
separate them:

  * unequal effective wheel diameters (Ed) -- the robot curves the same way
    in both directions, so the two clusters of end points land on OPPOSITE
    sides of the origin,
  * wrong effective wheelbase (Eb) -- the robot under- or over-turns each
    corner, so both clusters shift the SAME way.

The measure of quality is the distance from the origin to the centre of
gravity of each cluster:

    E_max = max( |cg_cw| , |cg_ccw| )

which is what gets quoted. The random component (the spread within a cluster)
is reported separately, because it is what SLAM has to cope with and what
calibration can never remove.

MEASUREMENT, HONESTLY
On hardware there is no ground truth. The end position must be measured with
a tape from the marked start point, and typed in. That is not a weakness of
the method: a tape measure on a floor is more trustworthy than the odometry
being calibrated, which is the whole point. In simulation the node can read
the truth topic instead and run unattended.
"""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

from youbot_commissioning.lib.stage import CommissioningStage, run


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class Stage2(CommissioningStage):
    STAGE = 2
    SLUG = "umbmark"
    TITLE = "UMBmark odometry calibration on the real floor"
    PROCEDURE = """
    BEFORE ARMING
      1. Stages 0 and 1 have PASSED.
      2. Choose a square of side L (default 2 m) on the SAME FLOOR the robot
         will work on. Not a corridor, not the lab: the greenhouse floor, wet
         if it will be wet in service. The floor is the experiment.
      3. Mark the start point and the start heading on the floor with tape.
         Mark a cross, not a dot -- you need to measure x and y separately.

    WHAT WILL HAPPEN
      The robot drives the square CLOCKWISE and returns near the start. It
      stops and waits. You measure, with a tape, how far the robot's reference
      point is from the cross, along the two marked axes, and publish those
      two numbers. Repeat 5 times. Then the same, counter-clockwise.

      To report a measurement:
        ros2 topic pub -1 /commissioning/measurement \\
          std_msgs/Float32MultiArray "{data: [dx, dy]}"     # metres

    WHAT IS BEING MEASURED
      Ed  : effective wheel-diameter ratio
      Eb  : effective wheelbase correction
      E_max : the UMBmark figure of merit, in metres

    PASS MEANS
      E_max is below the configured threshold AFTER correction, and the random
      spread is small enough that the localisation budget of stage 5 is
      feasible. A large E_max is not a failure of the robot -- it is a
      measurement you now have. A large random spread IS a problem, because
      calibration cannot remove it.
    """

    def __init__(self):
        super().__init__("stage2_umbmark")
        self.declare_parameter("side", 2.0)             # m
        self.declare_parameter("runs_per_direction", 5)
        self.declare_parameter("drive_speed", 0.15)     # m/s
        self.declare_parameter("turn_rate", 0.35)       # rad/s
        self.declare_parameter("use_truth", True)       # sim: read truth topic
        self.declare_parameter("truth_topic", "truth_pose")
        self.declare_parameter("max_emax", 0.15)        # m, after correction
        self.declare_parameter("max_random_spread", 0.10)  # m, std within a set

        self._side = float(self.get_parameter("side").value)
        self._n = int(self.get_parameter("runs_per_direction").value)
        self._use_truth = bool(self.get_parameter("use_truth").value)

        self._odom = None
        self._truth = None
        self._start_odom = None

        self._direction = "cw"
        self._run = 0
        self._leg = 0                 # 0..7: drive, turn, drive, turn, ...
        self._leg_start = None
        self._phase = "idle"          # idle | driving | turning | waiting
        self._cw: list[tuple[float, float]] = []
        self._ccw: list[tuple[float, float]] = []

        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter("truth_topic").value),
            self._on_truth, 10)
        self.create_subscription(Float32MultiArray, "/commissioning/measurement",
                                 self._on_measurement, 10)
        self.create_timer(0.05, self._tick)

    # --- inputs --------------------------------------------------------
    def _on_odom(self, msg: Odometry) -> None:
        self._odom = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                      _yaw(msg.pose.pose.orientation))

    def _on_truth(self, msg: PoseStamped) -> None:
        self._truth = (msg.pose.position.x, msg.pose.position.y,
                       _yaw(msg.pose.orientation))

    def _on_measurement(self, msg: Float32MultiArray) -> None:
        if self._phase != "waiting" or len(msg.data) < 2:
            return
        self._store(float(msg.data[0]), float(msg.data[1]), "tape")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # --- sequence ------------------------------------------------------
    def on_armed(self) -> None:
        self._start_run()

    def _start_run(self) -> None:
        if self._odom is None:
            self.get_logger().warn("waiting for /odom before starting")
            return
        self._start_odom = self._odom
        self._truth_start = self._truth
        self._leg = 0
        self._leg_start = self._odom
        self._phase = "driving"
        self.get_logger().info(
            f"--- {self._direction.upper()} run {self._run + 1}/{self._n}: "
            f"square of {self._side:.2f} m")

    def _tick(self) -> None:
        if not self.armed or self._odom is None or self._phase == "idle":
            return
        if self._phase == "waiting":
            self.stop()
            return

        sign = -1.0 if self._direction == "cw" else 1.0
        x, y, yaw = self._odom
        lx, ly, lyaw = self._leg_start

        if self._phase == "driving":
            gone = math.hypot(x - lx, y - ly)
            if gone >= self._side:
                self._leg += 1
                self._leg_start = self._odom
                self._phase = "turning"
                return
            self.publish_cmd(float(self.get_parameter("drive_speed").value),
                             0.0, 0.0)

        elif self._phase == "turning":
            turned = abs(_wrap(yaw - lyaw))
            if turned >= math.pi / 2.0 - 0.02:
                self._leg += 1
                self._leg_start = self._odom
                if self._leg >= 8:
                    self._finish_run()
                else:
                    self._phase = "driving"
                return
            self.publish_cmd(0.0, 0.0,
                             sign * float(self.get_parameter("turn_rate").value))

    def _finish_run(self) -> None:
        self.stop()
        if self._use_truth and self._truth is not None and self._truth_start:
            dx = self._truth[0] - self._truth_start[0]
            dy = self._truth[1] - self._truth_start[1]
            self._store(dx, dy, "truth")
        else:
            self._phase = "waiting"
            self.get_logger().info(
                "RUN COMPLETE. Measure dx and dy from the cross, in the frame "
                "you marked, and publish:\n"
                "  ros2 topic pub -1 /commissioning/measurement "
                "std_msgs/Float32MultiArray \"{data: [dx, dy]}\"")

    def _store(self, dx: float, dy: float, source: str) -> None:
        bucket = self._cw if self._direction == "cw" else self._ccw
        bucket.append((dx, dy))
        self.get_logger().info(
            f"  {self._direction} run {self._run + 1}: "
            f"dx={dx:+.3f} dy={dy:+.3f} m  (source: {source})")
        self._run += 1
        if self._run < self._n:
            self._phase = "idle"
            self._start_run()
        elif self._direction == "cw":
            self._direction = "ccw"
            self._run = 0
            self._phase = "idle"
            self.get_logger().info(
                "clockwise set complete. Reposition the robot on the cross, "
                "facing the marked heading, then re-arm for the "
                "COUNTER-CLOCKWISE set.")
            self._armed = False
        else:
            self._conclude()

    # --- UMBmark arithmetic ---------------------------------------------
    @staticmethod
    def _cg(points):
        if not points:
            return None, None
        n = len(points)
        return (sum(p[0] for p in points) / n, sum(p[1] for p in points) / n)

    @staticmethod
    def _spread(points, cg):
        if not points or cg[0] is None:
            return None
        return math.sqrt(sum((p[0] - cg[0]) ** 2 + (p[1] - cg[1]) ** 2
                             for p in points) / len(points))

    def _conclude(self) -> None:
        cg_cw = self._cg(self._cw)
        cg_ccw = self._cg(self._ccw)
        self.report.record("cw_endpoints", self._cw)
        self.report.record("ccw_endpoints", self._ccw)
        self.report.record("cg_cw", cg_cw)
        self.report.record("cg_ccw", cg_ccw)

        if cg_cw[0] is None or cg_ccw[0] is None:
            self.report.check("both directions measured", 0, "==", 1)
            self.finish()
            return

        r_cw = math.hypot(*cg_cw)
        r_ccw = math.hypot(*cg_ccw)
        e_max = max(r_cw, r_ccw)

        # Borenstein & Feng, eq. (10)-(13): the CG offsets separate into a
        # turn error (same sign both ways) and a curvature error (opposite).
        L = self._side
        alpha = ((cg_cw[0] + cg_ccw[0]) / (-4.0 * L)) if L else 0.0   # rad
        beta = ((cg_cw[0] - cg_ccw[0]) / (-4.0 * L)) if L else 0.0    # rad
        # Effective wheelbase correction and wheel-diameter ratio.
        b_nominal = float(self.get_parameter("base_width").value)
        eb = (math.pi / 2.0) / ((math.pi / 2.0) - alpha) if alpha else 1.0
        radius = (L / 2.0) / math.sin(beta / 2.0) if abs(beta) > 1e-9 else None
        ed = ((radius + b_nominal / 2.0) / (radius - b_nominal / 2.0)
              if radius and abs(radius) > b_nominal else 1.0)

        spread_cw = self._spread(self._cw, cg_cw)
        spread_ccw = self._spread(self._ccw, cg_ccw)
        spread = max(s for s in (spread_cw, spread_ccw) if s is not None)

        self.report.record("alpha_rad", alpha)
        self.report.record("beta_rad", beta)
        self.report.record("Eb_wheelbase_ratio", eb)
        self.report.record("Ed_diameter_ratio", ed)
        self.report.record("E_max_m", e_max)
        self.report.record("random_spread_m", spread)
        self.report.note(
            f"Apply Eb={eb:.5f} and Ed={ed:.5f} to the odometry, then RE-RUN "
            "this stage. The threshold below applies to the corrected robot; "
            "the first pass is expected to fail and that is the measurement.")
        self.report.note(
            "The random spread is what calibration CANNOT remove. It sets the "
            "process-noise floor for the EKF of stage 5 and it is the honest "
            "answer to 'how long can this robot run on odometry alone'.")

        self.report.check("clockwise runs", len(self._cw), ">=", self._n)
        self.report.check("counter-clockwise runs", len(self._ccw), ">=", self._n)
        self.report.check("E_max", e_max, "<=",
                          float(self.get_parameter("max_emax").value), "m",
                          "UMBmark figure of merit after correction")
        self.report.check("random spread", spread, "<=",
                          float(self.get_parameter("max_random_spread").value),
                          "m", "irreducible; drives the SLAM requirement")
        self.finish()


def main(args=None) -> None:
    run(Stage2)


if __name__ == "__main__":
    main()
