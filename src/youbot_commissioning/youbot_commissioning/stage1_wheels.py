"""Stage 1 -- wheel command, alone, with no mission on top.

The three canonical twists of a holonomic base:

    forward   (vx > 0, vy = 0, wz = 0)  -> the body must travel along its own
                                           +x axis and NOT rotate
    strafe    (vx = 0, vy > 0, wz = 0)  -> along +y, and NOT rotate
    yaw       (vx = 0, vy = 0, wz > 0)  -> rotate in place, and NOT translate

This is the Phase A `base_test` controller, rewritten as an acceptance test
with numeric criteria. It exists because every mecanum sign error in this
project produced a plausible-looking motion that was wrong in one axis, and
the only cheap way to catch that is to command one axis at a time and measure
what actually happened.

Two failure signatures to know before running it:

  * "strafe drifts forward" -- a wheel pair is saturating asymmetrically, or
    a sign is wrong on one diagonal. On hardware it is more often a wheel
    mounted with the wrong roller handedness: mecanum wheels come in A and B
    and they are easy to swap. Look at the rollers before you touch the code.
  * "forward curves" -- unequal wheel radii or a slipping wheel. On a real
    floor this is normal and small; it is what stage 2 measures properly.
"""

from __future__ import annotations

import math

from nav_msgs.msg import Odometry

from youbot_commissioning.lib.stage import CommissioningStage, run


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Stage1(CommissioningStage):
    STAGE = 1
    SLUG = "wheels"
    TITLE = "Wheel command: the three canonical twists"
    PROCEDURE = """
    BEFORE ARMING
      1. Stage 0 has PASSED. Do not run this otherwise.
      2. Clear 3 m ahead, 2 m to the left, and 1 m all round for the spin.
      3. Mark the robot's start position on the floor with tape. You will
         want to see the drift with your own eyes, not only in the numbers.

    WHAT WILL HAPPEN
      Three motions, each preceded by a 3 s countdown, each stopping by itself:
        forward  0.6 m
        strafe   0.6 m to the LEFT
        yaw      one half turn in place

    WHAT IS BEING MEASURED
      For each motion: the distance travelled, the direction travelled
      relative to the body, and the amount of unwanted motion in the other
      two degrees of freedom.

    PASS MEANS
      Each motion goes where it was asked, within tolerance, and the
      cross-coupling stays small. If strafe drifts forward, CHECK THE WHEEL
      ROLLER HANDEDNESS before changing any code.
    """

    def __init__(self):
        super().__init__("stage1_wheels")
        self.declare_parameter("test_distance", 0.60)   # m
        self.declare_parameter("test_speed", 0.12)      # m/s
        self.declare_parameter("test_yaw", math.pi)     # rad
        self.declare_parameter("yaw_rate", 0.35)        # rad/s
        self.declare_parameter("direction_tol_deg", 8.0)
        self.declare_parameter("distance_tol", 0.10)    # fraction
        self.declare_parameter("coupling_tol", 0.12)    # m or rad of unwanted

        self._pose = None
        self._start = None
        self._tests = ["forward", "strafe", "yaw"]
        self._i = 0
        self._phase = "idle"        # idle | countdown | running | settle
        self._phase_t = 0.0
        self._results: dict[str, dict] = {}

        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_timer(0.05, self._tick)

    def _on_odom(self, msg: Odometry) -> None:
        self._pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                      _yaw(msg.pose.pose.orientation))

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_armed(self) -> None:
        self._begin()

    def _begin(self) -> None:
        if self._i >= len(self._tests):
            self._conclude()
            return
        self._phase, self._phase_t = "countdown", self._now()
        self.get_logger().info(
            f"--- test {self._i + 1}/3: {self._tests[self._i].upper()} "
            "-- starting in 3 s")

    def _tick(self) -> None:
        if not self.armed or self._pose is None or self._phase == "idle":
            return
        t = self._now()
        name = self._tests[self._i] if self._i < len(self._tests) else None

        if self._phase == "countdown":
            self.stop()
            if t - self._phase_t >= 3.0:
                self._start = self._pose
                self._phase, self._phase_t = "running", t
            return

        if self._phase == "settle":
            self.stop()
            if t - self._phase_t >= 1.5:
                self._measure(name)
                self._i += 1
                self._begin()
            return

        # --- running -----------------------------------------------------
        d = float(self.get_parameter("test_distance").value)
        v = float(self.get_parameter("test_speed").value)
        if name == "forward":
            travelled = math.hypot(self._pose[0] - self._start[0],
                                   self._pose[1] - self._start[1])
            if travelled >= d:
                self._phase, self._phase_t = "settle", t
                return
            self.publish_cmd(v, 0.0, 0.0)
        elif name == "strafe":
            travelled = math.hypot(self._pose[0] - self._start[0],
                                   self._pose[1] - self._start[1])
            if travelled >= d:
                self._phase, self._phase_t = "settle", t
                return
            self.publish_cmd(0.0, v, 0.0)
        elif name == "yaw":
            turned = abs(_wrap(self._pose[2] - self._start[2]))
            target = float(self.get_parameter("test_yaw").value)
            # abs(wrap) folds at pi, so stop just short of a half turn.
            if turned >= min(target, math.pi - 0.05):
                self._phase, self._phase_t = "settle", t
                return
            self.publish_cmd(0.0, 0.0,
                             float(self.get_parameter("yaw_rate").value))

    # --- measurement ---------------------------------------------------
    def _measure(self, name: str) -> None:
        sx, sy, syaw = self._start
        px, py, pyaw = self._pose
        dx_w, dy_w = px - sx, py - sy
        # Express the displacement in the BODY frame the motion started in.
        c, s = math.cos(-syaw), math.sin(-syaw)
        dx_b = c * dx_w - s * dy_w
        dy_b = s * dx_w + c * dy_w
        dist = math.hypot(dx_b, dy_b)
        dyaw = _wrap(pyaw - syaw)

        res = {"body_dx": dx_b, "body_dy": dy_b, "distance": dist,
               "d_yaw": dyaw}
        if dist > 1e-3:
            res["direction_deg"] = math.degrees(math.atan2(dy_b, dx_b))
        self._results[name] = res
        self.get_logger().info(
            f"  {name}: body dx={dx_b:+.3f} dy={dy_b:+.3f} "
            f"|d|={dist:.3f} m, dyaw={math.degrees(dyaw):+.1f} deg")

    def _conclude(self) -> None:
        self.report.record("motions", self._results)
        d = float(self.get_parameter("test_distance").value)
        dtol = float(self.get_parameter("distance_tol").value)
        atol = float(self.get_parameter("direction_tol_deg").value)
        ctol = float(self.get_parameter("coupling_tol").value)

        fwd = self._results.get("forward", {})
        stf = self._results.get("strafe", {})
        yaw = self._results.get("yaw", {})

        # Forward: travels along +x, does not turn, does not slide sideways.
        self.report.check("forward: distance", fwd.get("distance"), ">=",
                          d * (1 - dtol), "m")
        self.report.check("forward: direction error",
                          abs(fwd.get("direction_deg", 999.0)), "<=", atol,
                          "deg", "0 deg means straight along the body +x axis")
        self.report.check("forward: unwanted sideways",
                          abs(fwd.get("body_dy", 99.0)), "<=", ctol, "m")
        self.report.check("forward: unwanted rotation",
                          abs(math.degrees(fwd.get("d_yaw", 99.0))), "<=",
                          math.degrees(ctol) * 0.5, "deg")

        # Strafe: travels along +y (i.e. 90 deg), does not turn.
        strafe_err = abs(_wrap(math.radians(stf.get("direction_deg", 999.0))
                               - math.pi / 2.0))
        self.report.check("strafe: distance", stf.get("distance"), ">=",
                          d * (1 - dtol), "m")
        self.report.check("strafe: direction error",
                          math.degrees(strafe_err), "<=", atol, "deg",
                          "if this fails, INSPECT THE WHEEL ROLLER HANDEDNESS "
                          "before editing the kinematics")
        self.report.check("strafe: unwanted forward",
                          abs(stf.get("body_dx", 99.0)), "<=", ctol, "m")

        # Yaw: rotates, does not translate.
        self.report.check("yaw: rotated",
                          abs(math.degrees(yaw.get("d_yaw", 0.0))), ">=", 150.0,
                          "deg")
        self.report.check("yaw: unwanted translation",
                          yaw.get("distance"), "<=", ctol, "m",
                          "a base that walks while spinning has an off-centre "
                          "rotation point, usually a wrong l_x or l_y")
        self.finish()


def main(args=None) -> None:
    run(Stage1)


if __name__ == "__main__":
    main()
