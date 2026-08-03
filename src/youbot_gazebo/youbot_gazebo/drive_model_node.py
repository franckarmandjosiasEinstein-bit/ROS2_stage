"""drive_model_node -- the drivetrain the simulation does not have.

WHERE THIS SITS, AND WHY IT IS NOT A CONTROLLER

    planners -> /cmd_vel_raw -> safety_node -> /cmd_vel
                                                  |
                                          [ THIS NODE ]   <- the PLANT
                                                  |
                                            /cmd_vel_exec -> Gazebo

It is deliberately on the Gazebo side of the line. It models MOTORS, not
policy: single ownership of /cmd_vel is untouched, safety_node is still the
only writer of it, and nothing here ever decides where the robot should go.
Switching it off is a pass-through and the stack behaves exactly as before.

WHAT IT IS FOR

Gazebo's VelocityControl is kinematic: you command a body twist and you get
it, instantly and exactly. That is a fine assumption for developing a mission
layer and a hopeless one for two things this project now needs.

  1. Sim-to-real. Every number measured against a kinematic plant is an
     optimistic bound. A stack tuned there meets a real drivetrain for the
     first time in a greenhouse, which is the worst place to discover
     acceleration limits.
  2. Predictive control. An MPC's whole value is anticipating dynamics and
     respecting constraints. Against a plant with neither, it is provably
     equivalent to the geometric controller it was meant to beat -- the work
     gets done and cannot be shown to be worth doing.

WHAT IS MODELLED, AND WHY EACH ONE MATTERS

  transport delay   sensing, DDS, the control period and the drive's own
                    input latency. This is the t_react of ISO 13855 and it
                    is what makes a protective distance more than geometry.

  per-wheel loop    the body twist is resolved through the MECANUM INVERSE
                    KINEMATICS first, and the lag, rate limit and saturation
                    are applied TO EACH WHEEL, then mapped back through the
                    forward kinematics. This is the point of the whole node.
                    Saturating a body twist scales it; saturating one wheel
                    of a diagonal pair DISTORTS it -- ask for forward+yaw
                    beyond the envelope and a real base gives you a curve
                    you did not command. That coupling cannot be reproduced
                    by adding noise to (vx, vy, wz), and it is exactly the
                    kind of thing constrained MPC exists to handle.

  first-order lag   a geared DC drive under a velocity loop does not step.
                    tau is small (tens of ms) but at 20 Hz it is a whole
                    control period, which is where the follower's phase
                    margin goes.

  rate limit        finite torque. Commanding a step is what makes mecanum
                    rollers break traction in the first place.

  slip              the encoders report the commanded wheel motion; the body
                    does something slightly different. That difference IS
                    odometry drift -- not a random walk added to a pose, but
                    the physical mechanism. So this node publishes both:
                    /wheel_speeds (what the encoders would say) and the
                    achieved twist (what the body does). Integrating the
                    former while the latter is true reproduces real dead
                    reckoning, and makes the stage-2 UMBmark calibration
                    meaningful in simulation instead of a formality.

DEFAULTS ARE OFF. `enabled:=false` is a pure pass-through. The realism is
opt-in so the measured baseline of the report stays reproducible and so a
regression in the mission layer can never be blamed on the plant changing
underneath it. Turn it on to measure the cost of reality:

    ros2 launch youbot_gazebo gazebo.launch.py drive_model:=true
"""

from __future__ import annotations

import math
import random
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


# YouBot geometry, identical to youbot_control.lib and the Webots controller.
# Duplicated deliberately: this node models the HARDWARE and must not import
# the control stack, or the plant and the controller would share a bug.
WHEEL_RADIUS = 0.05
LX = 0.228
LY = 0.158
L = LX + LY          # 0.386


def body_to_wheels(vx: float, vy: float, wz: float) -> list[float]:
    """Mecanum inverse kinematics, native wheel order [w1..w4], rad/s."""
    r = WHEEL_RADIUS
    return [
        (vx + vy + L * wz) / r,     # front left
        (vx - vy - L * wz) / r,     # front right
        (vx - vy + L * wz) / r,     # rear left
        (vx + vy - L * wz) / r,     # rear right
    ]


def wheels_to_body(w: list[float]) -> tuple[float, float, float]:
    """Forward kinematics: the twist four wheel speeds actually produce."""
    r = WHEEL_RADIUS
    vx = r * (w[0] + w[1] + w[2] + w[3]) / 4.0
    vy = r * (w[0] - w[1] - w[2] + w[3]) / 4.0
    wz = r * (w[0] - w[1] + w[2] - w[3]) / (4.0 * L)
    return vx, vy, wz


class DriveModelNode(Node):
    def __init__(self) -> None:
        super().__init__("drive_model_node")

        self.declare_parameter("enabled", False)
        self.declare_parameter("rate", 50.0)          # model integration rate

        # --- actuation ------------------------------------------------
        # tau: a geared DC drive closing its own velocity loop settles in
        # roughly 0.2-0.3 s; tau = 0.08 s puts 95% of that inside 0.24 s.
        self.declare_parameter("tau", 0.08)           # s, first-order lag
        # 25 rad/s^2 at r = 0.05 m is 1.25 m/s^2 at the rim, a conservative
        # figure for a ~20 kg platform on a smooth floor. Stage 1 on hardware
        # replaces it with a measurement.
        self.declare_parameter("max_wheel_accel", 25.0)   # rad/s^2
        self.declare_parameter("max_wheel_speed", 14.0)   # rad/s
        # Scan age + DDS + one control period + drive input latency. This is
        # the t_react that config/limits.yaml assumes; keep the two in step.
        self.declare_parameter("command_delay", 0.06)     # s

        # --- slip ------------------------------------------------------
        # Mecanum rollers slip by construction and asymmetrically: rolling
        # forward is nearly clean, strafing loads every roller sideways, and
        # rotation does both at once. These are fractions of commanded motion.
        self.declare_parameter("slip_longitudinal", 0.015)
        self.declare_parameter("slip_lateral", 0.060)
        self.declare_parameter("slip_rotational", 0.090)
        self.declare_parameter("slip_noise", 0.010)   # per-wheel random part
        self.declare_parameter("seed", 0)             # 0 = time-seeded

        self.declare_parameter("in_topic", "cmd_vel")
        self.declare_parameter("out_topic", "cmd_vel_exec")

        self._enabled = bool(self.get_parameter("enabled").value)
        self._dt = 1.0 / float(self.get_parameter("rate").value)
        self._omega_max = float(self.get_parameter("max_wheel_speed").value)

        seed = int(self.get_parameter("seed").value)
        self._rng = random.Random(seed if seed else None)

        self._queue: deque[tuple[float, tuple[float, float, float]]] = deque()
        self._want = (0.0, 0.0, 0.0)       # latest command, after the delay
        self._wheels = [0.0, 0.0, 0.0, 0.0]   # current wheel speeds (encoders)
        self._last_cmd_t = 0.0
        self._saturated_ticks = 0
        self._ticks = 0

        self.create_subscription(
            Twist, str(self.get_parameter("in_topic").value), self._on_cmd, 10)
        self.exec_pub = self.create_publisher(
            Twist, str(self.get_parameter("out_topic").value), 10)
        # What the encoders would report. An odometry integrator fed from here
        # drifts for the right reason instead of by injected noise.
        self.wheel_pub = self.create_publisher(
            Float32MultiArray, "wheel_speeds", 10)
        # Commanded minus achieved body twist, so the cost of the drivetrain
        # is observable rather than inferred.
        self.err_pub = self.create_publisher(
            Float32MultiArray, "drive_model/twist_error", 10)

        self.create_timer(self._dt, self._tick)

        if self._enabled:
            self.get_logger().info(
                "drive_model_node ACTIVE: tau=%.3f s, accel<=%.0f rad/s^2, "
                "|w|<=%.1f rad/s, delay=%.0f ms, slip %.1f/%.1f/%.1f%% "
                "(long/lat/rot). The base now has a drivetrain."
                % (float(self.get_parameter("tau").value),
                   float(self.get_parameter("max_wheel_accel").value),
                   self._omega_max,
                   1000.0 * float(self.get_parameter("command_delay").value),
                   100.0 * float(self.get_parameter("slip_longitudinal").value),
                   100.0 * float(self.get_parameter("slip_lateral").value),
                   100.0 * float(self.get_parameter("slip_rotational").value)))
        else:
            self.get_logger().info(
                "drive_model_node PASS-THROUGH (enabled:=false). The base is "
                "kinematic, as before. Launch with drive_model:=true to give "
                "it inertia, latency, saturation and slip.")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg: Twist) -> None:
        cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not self._enabled:
            self.exec_pub.publish(msg)          # byte-for-byte pass-through
            return
        delay = float(self.get_parameter("command_delay").value)
        self._queue.append((self._now() + delay, cmd))
        self._last_cmd_t = self._now()

    def _tick(self) -> None:
        if not self._enabled:
            return
        t = self._now()

        # --- 1. transport delay ---------------------------------------
        while self._queue and self._queue[0][0] <= t:
            self._want = self._queue.popleft()[1]

        # Command timeout: silence means stop, never coast. Mirrors the
        # cmd_timeout of safety_node, because a real drive does the same.
        if t - self._last_cmd_t > 1.0:
            self._want = (0.0, 0.0, 0.0)

        # --- 2. into wheel space BEFORE any limiting ------------------
        target = body_to_wheels(*self._want)

        tau = max(1e-3, float(self.get_parameter("tau").value))
        alpha = 1.0 - math.exp(-self._dt / tau)      # exact first-order step
        max_dw = float(self.get_parameter("max_wheel_accel").value) * self._dt

        saturated = False
        for i in range(4):
            # first-order lag toward the target
            step = alpha * (target[i] - self._wheels[i])
            # finite torque: the step itself is rate limited
            if abs(step) > max_dw:
                step = math.copysign(max_dw, step)
                saturated = True
            w = self._wheels[i] + step
            # speed ceiling, per wheel -- this is what distorts the twist
            if abs(w) > self._omega_max:
                w = math.copysign(self._omega_max, w)
                saturated = True
            self._wheels[i] = w

        self._ticks += 1
        if saturated:
            self._saturated_ticks += 1
            if self._saturated_ticks % 100 == 1:
                self.get_logger().warn(
                    "wheel limits active: the achieved twist is not the "
                    "commanded one (%.0f%% of ticks so far). This is the "
                    "coupling a kinematic plant hides."
                    % (100.0 * self._saturated_ticks / max(1, self._ticks)),
                    throttle_duration_sec=5.0)

        # --- 3. what the encoders report ------------------------------
        msg = Float32MultiArray()
        msg.data = [float(w) for w in self._wheels]
        self.wheel_pub.publish(msg)

        # --- 4. slip: the body does slightly less than the wheels say --
        vx, vy, wz = wheels_to_body(self._wheels)
        g = self.get_parameter
        s_lon = float(g("slip_longitudinal").value)
        s_lat = float(g("slip_lateral").value)
        s_rot = float(g("slip_rotational").value)
        noise = float(g("slip_noise").value)

        def jitter() -> float:
            return self._rng.gauss(0.0, noise) if noise > 0.0 else 0.0

        # Rotation loads every roller, so it degrades the translation axes
        # too: a base that is turning tracks worse than one going straight.
        rot_load = min(1.0, abs(wz) / 1.0)
        vx *= (1.0 - s_lon - s_rot * rot_load * 0.5 + jitter())
        vy *= (1.0 - s_lat - s_rot * rot_load * 0.5 + jitter())
        wz *= (1.0 - s_rot + jitter())

        out = Twist()
        out.linear.x, out.linear.y, out.angular.z = vx, vy, wz
        self.exec_pub.publish(out)

        err = Float32MultiArray()
        err.data = [float(self._want[0] - vx), float(self._want[1] - vy),
                    float(self._want[2] - wz)]
        self.err_pub.publish(err)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DriveModelNode()
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
