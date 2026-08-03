"""Stage 0 -- emergency stop and speed limiting.

NOTHING ELSE IN THIS PACKAGE MAY BE RUN BEFORE THIS STAGE PASSES.

What is being tested is not software. It is the hardware chain: a mushroom
button that cuts motor power without the computer's participation. This node
cannot verify that chain by itself -- if it could, the chain would be going
through software, which is exactly what must not happen. What it CAN do is:

  * command a known, slow motion and measure how long the wheels keep
    turning after the operator hits the button (from /odom or /joint_states,
    whichever survives the cut),
  * verify that the commissioning speed ceiling is actually enforced,
  * verify that the base stops on command timeout, so a crashed process or a
    severed network link leaves the robot stationary rather than coasting.

The stopping time measured here is not a formality. It feeds the protective
distance of every later stage through ISO 13855:

    S = K * (t_react + t_stop) + C

with K the approach speed. Stage 6 will not be allowed to run until this
number exists, because a navigation clearance chosen without it is a guess.
"""

from __future__ import annotations

import math

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from youbot_commissioning.lib.stage import CommissioningStage, run


class Stage0(CommissioningStage):
    STAGE = 0
    SLUG = "estop"
    TITLE = "Emergency stop and speed limiting"
    PROCEDURE = """
    BEFORE ARMING
      1. The robot is on blocks or in a clear area at least 3 m across.
      2. The hardware E-stop is within reach of the operator AND of a second
         person. Test it once now, with the motors unpowered, to feel where it
         is in the dark.
      3. Nobody stands in front of the robot. Ever, during this stage.

    WHAT WILL HAPPEN
      The robot drives slowly FORWARD in bursts. On each burst you press the
      E-stop. The node measures how far and how long the wheels kept moving
      after the cut. Release the E-stop between bursts and re-arm.

    WHAT IS BEING MEASURED
      * stopping time and distance after the hardware cut
      * whether the commanded speed ever exceeded the ceiling
      * whether the base stops on its own when commands stop arriving

    PASS MEANS
      The wheels stop within the configured stopping time, every time, and the
      timeout stop works. If any single burst fails, the stage fails: a
      protective stop that works four times out of five does not work.
    """

    def __init__(self):
        super().__init__("stage0_estop")
        self.declare_parameter("bursts", 3)
        self.declare_parameter("burst_speed", 0.15)      # m/s
        self.declare_parameter("max_stop_time", 0.60)    # s after the cut
        self.declare_parameter("max_stop_distance", 0.15)  # m after the cut
        self.declare_parameter("timeout_stop_wait", 2.0)  # s of silence

        self._bursts = int(self.get_parameter("bursts").value)
        self._burst_speed = float(self.get_parameter("burst_speed").value)

        self._odom = None            # (x, y, speed, stamp)
        self._results: list[dict] = []
        self._phase = "idle"         # idle | driving | stopping | timeout | done
        self._phase_t = 0.0
        self._cut_state = None       # pose + time at the moment of the cut
        self._moving_since = None

        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        # The operator asserts the cut here so the node knows WHEN to start
        # measuring. This topic does not perform the cut -- the button does.
        self.create_subscription(Bool, "/commissioning/estop_pressed",
                                 self._on_estop, 1)
        self.create_timer(0.02, self._tick)   # 50 Hz: we are timing a stop

    # --- inputs --------------------------------------------------------
    def _on_odom(self, msg: Odometry) -> None:
        v = msg.twist.twist.linear
        self._odom = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                      math.hypot(v.x, v.y), self._now())

    def _on_estop(self, msg: Bool) -> None:
        if not msg.data or self._phase != "driving":
            return
        if self._odom is None:
            self.get_logger().warn("E-stop pressed but no odometry -- "
                                   "cannot measure the stop.")
            return
        self._cut_state = self._odom
        self._phase, self._phase_t = "stopping", self._now()
        self.get_logger().warn("E-STOP asserted -- measuring the stop.")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # --- sequence ------------------------------------------------------
    def on_armed(self) -> None:
        self._phase, self._phase_t = "driving", self._now()
        self.get_logger().info(
            f"burst {len(self._results) + 1}/{self._bursts}: driving forward "
            f"at {self._burst_speed:.2f} m/s -- PRESS THE E-STOP, then publish "
            "/commissioning/estop_pressed true")

    def _tick(self) -> None:
        t = self._now()

        if self._phase == "driving":
            self.publish_cmd(self._burst_speed, 0.0, 0.0)
            if t - self._phase_t > 30.0:
                self.get_logger().warn("no E-stop within 30 s -- aborting the "
                                       "burst and stopping.")
                self.stop()
                self._phase = "idle"

        elif self._phase == "stopping":
            # Keep publishing the SAME command. The point is that the hardware
            # cut wins over a controller that is still asking for motion --
            # which is the only version of the test that means anything.
            self.publish_cmd(self._burst_speed, 0.0, 0.0)
            if self._odom is None:
                return
            x, y, speed, _ = self._odom
            if speed < 0.01:
                cx, cy, _, ct = self._cut_state
                dt = t - ct
                dd = math.hypot(x - cx, y - cy)
                self._results.append({"stop_time_s": dt, "stop_distance_m": dd})
                self.get_logger().info(
                    f"  stopped in {dt:.3f} s over {dd:.3f} m")
                self.stop()
                if len(self._results) >= self._bursts:
                    self._phase, self._phase_t = "timeout", t
                    self.get_logger().info(
                        "all bursts done -- now testing the COMMAND TIMEOUT: "
                        "release the E-stop, the node will drive briefly then "
                        "go silent, and the base must stop by itself.")
                else:
                    self._phase = "idle"
                    self.get_logger().info(
                        "release the E-stop and re-arm for the next burst "
                        "(publish /commissioning/arm false then true)")
                    self._armed = False
            elif t - self._phase_t > 5.0:
                self.get_logger().error(
                    "the wheels are STILL turning 5 s after the cut. "
                    "The E-stop chain does not work. Stop here.")
                self._results.append({"stop_time_s": 5.0,
                                      "stop_distance_m": float("nan")})
                self.stop()
                self._phase = "done"
                self._conclude()

        elif self._phase == "timeout":
            # Drive for 1 s, then deliberately publish nothing at all.
            if t - self._phase_t < 1.0:
                self.publish_cmd(self._burst_speed, 0.0, 0.0)
                self._moving_since = t
            else:
                wait = float(self.get_parameter("timeout_stop_wait").value)
                if self._odom is not None and self._odom[2] < 0.01:
                    self.report.record("timeout_stop_ok", True)
                    self._phase = "done"
                    self._conclude()
                elif t - self._phase_t > 1.0 + wait:
                    self.report.record("timeout_stop_ok", False)
                    self._phase = "done"
                    self._conclude()

    # --- verdict -------------------------------------------------------
    def _conclude(self) -> None:
        times = [r["stop_time_s"] for r in self._results]
        dists = [r["stop_distance_m"] for r in self._results
                 if not math.isnan(r["stop_distance_m"])]
        self.report.record("bursts", self._results)

        worst_t = max(times) if times else None
        worst_d = max(dists) if dists else None

        self.report.check("every burst was measured", len(self._results),
                          ">=", self._bursts, "bursts")
        self.report.check("worst stopping time", worst_t, "<=",
                          float(self.get_parameter("max_stop_time").value), "s",
                          "measured from the moment the operator asserted the cut")
        self.report.check("worst stopping distance", worst_d, "<=",
                          float(self.get_parameter("max_stop_distance").value),
                          "m")
        self.report.check("base stops on command timeout",
                          1 if self.report.data.get("timeout_stop_ok") else 0,
                          "==", 1)

        # The number every later stage needs.
        if worst_t is not None and worst_d is not None:
            self.report.record("iso13855_t_stop_s", worst_t)
            self.report.note(
                "ISO 13855 protective distance at 0.6 m/s with a 0.20 s "
                f"reaction time: S = 0.6*(0.20+{worst_t:.2f}) + C = "
                f"{0.6 * (0.20 + worst_t):.2f} m before the margin C. "
                "Stage 6 must use at least this as its stop distance.")
        self.finish()


def main(args=None) -> None:
    run(Stage0)


if __name__ == "__main__":
    main()
