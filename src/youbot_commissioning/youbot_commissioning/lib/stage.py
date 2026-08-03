"""stage -- the base class every commissioning stage node inherits.

Three things every stage needs and none of them is the stage's own subject:

1. A SPEED CEILING that is enforced here, not trusted to the stage. Stage 1
   drives the wheels, stage 6 drives a whole mission; both must be incapable
   of exceeding the commissioning speed limit, and the limit must not depend
   on the stage remembering to apply it. `self.publish_cmd()` is the only way
   a stage sends a twist, and it clamps.

2. An OPERATOR PROMPT. Field commissioning is a person and a robot in a
   greenhouse. A stage that starts moving as soon as it launches is dangerous
   and useless. Every stage prints its procedure, then waits for the operator
   to arm it on /commissioning/arm, and stops the moment that goes false.

3. THE PROFILE. Every physical number (speed ceiling, geometry, expected
   stopping distance) comes from config/limits.yaml, selected by the
   `profile` parameter: "sim" or "hardware". A stage never hard-codes a
   distance -- that is exactly the drift that cost this project a whole run
   when youbot_params.yaml and the node defaults disagreed.

Safety note that belongs in the code and not only in the README: the arm
topic and this clamp are FUNCTIONAL safety. They are Python in a non
real-time process and they are not a protective device. The hardware E-stop
of stage 0 is what protects people; this class only makes the tests
well-behaved.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from youbot_commissioning.lib.report import Report


class CommissioningStage(Node):
    #: subclasses set these
    STAGE = -1
    SLUG = "unnamed"
    TITLE = "unnamed stage"
    #: printed before the operator arms the stage. Write it as if the reader
    #: is standing next to the robot holding the E-stop, because they are.
    PROCEDURE = "No procedure written for this stage."

    def __init__(self, name: str):
        super().__init__(name)

        self.declare_parameter("profile", "sim")     # "sim" | "hardware"
        self.declare_parameter("cmd_topic", "cmd_vel_raw")
        self.declare_parameter("max_linear", 0.20)   # m/s ceiling for tests
        self.declare_parameter("max_angular", 0.50)  # rad/s ceiling
        self.declare_parameter("require_arm", True)
        self.declare_parameter("base_length", 0.58)
        self.declare_parameter("base_width", 0.38)

        self.profile = str(self.get_parameter("profile").value)
        self._vmax = float(self.get_parameter("max_linear").value)
        self._wmax = float(self.get_parameter("max_angular").value)
        self._require_arm = bool(self.get_parameter("require_arm").value)

        self.report = Report(self.STAGE, self.SLUG, self.TITLE,
                             logger=self.get_logger(),
                             platform_name=self.profile)

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_topic").value), 10)
        # The operator's switch. Publish once to start, again to abort:
        #   ros2 topic pub -1 /commissioning/arm std_msgs/Bool "{data: true}"
        self._armed = not self._require_arm
        self.create_subscription(Bool, "/commissioning/arm", self._on_arm, 1)
        # Free-text field notes typed from another terminal end up in the
        # report, so what the operator saw is stored with what was measured.
        self.create_subscription(String, "/commissioning/note",
                                 self._on_note, 10)

        self._finished = False
        self._clamped_count = 0

        self._print_procedure()

    # --- operator interface -------------------------------------------
    def _print_procedure(self) -> None:
        log = self.get_logger()
        bar = "=" * 68
        log.info(bar)
        log.info(f"STAGE {self.STAGE} -- {self.TITLE}   [profile: {self.profile}]")
        log.info(bar)
        for line in self.PROCEDURE.strip().splitlines():
            log.info(line.rstrip())
        log.info("-" * 68)
        log.info(f"speed ceiling for this stage: {self._vmax:.2f} m/s, "
                 f"{self._wmax:.2f} rad/s")
        if self._require_arm:
            log.info("ARM WHEN READY:  ros2 topic pub -1 /commissioning/arm "
                     "std_msgs/Bool \"{data: true}\"")
            log.info("ABORT AT ANY TIME:  same command with data: false "
                     "(and use the hardware E-stop first)")
        log.info(bar)

    def _on_arm(self, msg: Bool) -> None:
        was = self._armed
        self._armed = bool(msg.data)
        if self._armed and not was:
            self.get_logger().info("ARMED -- the stage is now driving.")
            self.on_armed()
        elif was and not self._armed:
            self.get_logger().warn("DISARMED -- stopping and holding.")
            self.stop()
            self.on_disarmed()

    def _on_note(self, msg: String) -> None:
        self.report.note(f"operator: {msg.data}")

    @property
    def armed(self) -> bool:
        return self._armed

    # --- hooks for subclasses -----------------------------------------
    def on_armed(self) -> None:
        """Called once when the operator arms the stage."""

    def on_disarmed(self) -> None:
        """Called when the operator disarms mid-run."""

    # --- the only way to command the base ------------------------------
    def publish_cmd(self, vx: float, vy: float = 0.0, wz: float = 0.0) -> None:
        """Publish a twist, clamped to the commissioning ceiling.

        The linear clamp scales BOTH axes by one factor rather than
        truncating each, so a diagonal command keeps its direction and only
        loses magnitude -- the same rule as the mecanum wheel saturation and
        the protective stop. A clamp that changes the direction of a test
        command invalidates the test.
        """
        if not self._armed:
            vx = vy = wz = 0.0
        speed = math.hypot(vx, vy)
        if speed > self._vmax > 0.0:
            scale = self._vmax / speed
            vx, vy = vx * scale, vy * scale
            self._clamped_count += 1
        if abs(wz) > self._wmax:
            wz = math.copysign(self._wmax, wz)
            self._clamped_count += 1
        msg = Twist()
        msg.linear.x, msg.linear.y = float(vx), float(vy)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def stop(self) -> None:
        msg = Twist()
        self.cmd_pub.publish(msg)

    # --- ending --------------------------------------------------------
    def finish(self) -> None:
        """Stop the base, write the report, and shut the node down once."""
        if self._finished:
            return
        self._finished = True
        self.stop()
        if self._clamped_count:
            self.report.note(
                f"the speed ceiling clamped {self._clamped_count} commands; "
                "the stage asked for more than the commissioning limit allows")
        self.report.record("speed_ceiling_linear", self._vmax)
        self.report.record("speed_ceiling_angular", self._wmax)
        self.report.finish()
        # Give the zero twist a moment to leave before the context closes.
        self.create_timer(0.2, self._shutdown)

    def _shutdown(self) -> None:
        rclpy.shutdown()


def run(node_class) -> None:
    """Standard main() for a stage node: spin until it finishes or Ctrl-C.

    Ctrl-C still writes a report. A stage aborted halfway is a result --
    'inconclusive, stopped at lap 3' is information; a silent exit is not.
    """
    rclpy.init()
    node = node_class()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("interrupted by the operator")
        node.stop()
        node.report.note("stage interrupted by the operator (Ctrl-C)")
        node.report.finish()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
