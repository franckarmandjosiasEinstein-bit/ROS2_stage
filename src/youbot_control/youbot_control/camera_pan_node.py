"""camera_pan_node -- owns the +/- 90 deg camera head.

WHY THIS NODE EXISTS AT ALL

The camera used to be bolted to the chassis with a permanent +90 deg yaw, so
the detector could treat its bearing as a constant. A pan head buys coverage
-- both plant rows from one aisle instead of one row per pass -- and it costs
exactly one thing: the bearing is no longer constant, and it enters the fruit
back-projection IN SERIES with the robot yaw. A 5 deg error in the head is a
17 cm error on a fruit at 2 m, the same sensitivity as a 5 deg heading error.

So the head is given a single owner, which does three things and nothing else:

  1. It commands a HELD target, never a sweep. Continuous motion means every
     frame has a different, unknown angle.
  2. It reports the MEASURED angle from /joint_states, not the commanded one.
     A position controller has a settling time; the difference between what
     was asked and what the joint is doing is precisely the error that would
     otherwise land silently in the fruit map.
  3. It publishes a SETTLED flag, and the detector refuses to publish world
     positions while that flag is false. A frame taken mid-sweep is discarded
     rather than back-projected with a guessed angle.

DEFAULT BEHAVIOUR IS THE OLD BEHAVIOUR
The default target is +pi/2 -- looking left, at the +Y plant row -- and the
camera sits 0.14 m out along the head, so at that angle it occupies body
(0.18, 0.14, 0.78): the exact pose the fruit projection was validated against.
Adding the head therefore changes nothing until something commands it to move.
That is deliberate. A mechanism that degrades the working configuration the
day it is installed is not an improvement.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64


class CameraPanNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_pan_node")

        self.declare_parameter("joint_name", "j_camera_pan")
        # Gazebo's joint states, bridged to a name of their own.
        # NOT /joint_states: arm_node owns that one for RViz.
        self.declare_parameter("joint_states_topic", "gz_joint_states")
        self.declare_parameter("default_target", math.pi / 2.0)  # look left
        self.declare_parameter("limit", 1.5708)      # matches the URDF limit
        self.declare_parameter("settle_tolerance", 0.02)   # rad, ~1.1 deg
        self.declare_parameter("settle_ticks", 5)    # consecutive ticks inside
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("command_period", 0.5)  # s between re-commands

        self._joint = str(self.get_parameter("joint_name").value)
        self._js_topic = str(
            self.get_parameter("joint_states_topic").value)
        self._limit = abs(float(self.get_parameter("limit").value))
        self._target = self._clamp(
            float(self.get_parameter("default_target").value))
        self._measured = None
        self._inside = 0
        self._settled = False
        self._last_cmd = 0.0

        self.cmd_pub = self.create_publisher(Float64, "camera_pan_cmd", 5)
        # The angle the rest of the stack must use. Published only when it is
        # measured; nothing downstream should ever fall back to a constant.
        self.state_pub = self.create_publisher(Float64, "camera_pan_state", 5)
        self.settled_pub = self.create_publisher(Bool, "camera_pan_settled", 5)

        self.create_subscription(Float64, "camera_pan_target",
                                 self._on_target, 5)
        self.create_subscription(JointState, self._js_topic,
                                 self._on_joint_states, 10)

        self.create_timer(1.0 / float(self.get_parameter("rate").value),
                          self._tick)
        self.get_logger().info(
            f"camera_pan_node up: holding {math.degrees(self._target):+.1f} deg "
            f"(limit +/-{math.degrees(self._limit):.0f} deg). "
            "Publish /camera_pan_target (rad) to move the head.")

    def _clamp(self, a: float) -> float:
        return max(-self._limit, min(self._limit, a))

    def _on_target(self, msg: Float64) -> None:
        want = self._clamp(float(msg.data))
        if abs(want - float(msg.data)) > 1e-6:
            self.get_logger().warn(
                f"target {math.degrees(float(msg.data)):+.1f} deg is outside "
                f"the +/-{math.degrees(self._limit):.0f} deg travel; "
                f"clamped to {math.degrees(want):+.1f} deg")
        if abs(want - self._target) < 1e-6:
            return
        self._target = want
        self._inside = 0
        self._settled = False
        self.settled_pub.publish(Bool(data=False))
        self.get_logger().info(
            f"head moving to {math.degrees(self._target):+.1f} deg "
            "-- detections are suppressed until it settles")

    def _on_joint_states(self, msg: JointState) -> None:
        try:
            i = list(msg.name).index(self._joint)
        except ValueError:
            return
        if i < len(msg.position):
            self._measured = float(msg.position[i])

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9

        # Re-assert the target periodically rather than once: a controller that
        # missed the single command at start-up would otherwise hold zero
        # forever, and the failure would look like "the camera sees nothing".
        if now - self._last_cmd >= float(
                self.get_parameter("command_period").value):
            self.cmd_pub.publish(Float64(data=self._target))
            self._last_cmd = now

        if self._measured is None:
            # No joint feedback: say so, loudly and repeatedly, and publish
            # NOTHING. Publishing the commanded angle here would be the exact
            # mistake this node exists to prevent -- it would look like a
            # measurement and would silently corrupt every fruit position.
            self.settled_pub.publish(Bool(data=False))
            self.get_logger().warn(
                "no %s for the pan joint" % self._js_topic + " -- the head angle is "
                "unknown, so no camera pose is published and fruit "
                "localisation is suppressed", throttle_duration_sec=5.0)
            return

        self.state_pub.publish(Float64(data=self._measured))

        error = abs(self._measured - self._target)
        if error <= float(self.get_parameter("settle_tolerance").value):
            self._inside += 1
        else:
            self._inside = 0

        settled = self._inside >= int(self.get_parameter("settle_ticks").value)
        if settled != self._settled:
            self._settled = settled
            self.get_logger().info(
                f"head {'settled at' if settled else 'left'} "
                f"{math.degrees(self._measured):+.1f} deg")
        self.settled_pub.publish(Bool(data=self._settled))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraPanNode()
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
