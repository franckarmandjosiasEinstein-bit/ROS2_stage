"""noisy_odom -- turn the ground-truth /odom into realistic drifting odometry.

In simulation the bridged /odom is perfect (Gazebo's OdometryPublisher is
ground truth), so a SLAM layer has nothing to prove. Real mecanum platforms
are the opposite: wheel slip makes encoder odometry drift within metres.
This node simulates that reality so the SLAM correction is demonstrable:

    /odom (ground truth)  ->  [systematic scale bias + white noise,
                               integrated in the BODY frame]  ->  /odom_noisy

The corruption model follows real encoder behaviour:
  * a systematic scale bias per axis (mecanum lateral slip is the worst,
    so the Y axis gets the largest bias) -- this produces the smooth,
    ever-growing drift seen on real robots;
  * zero-mean white noise per step on top;
  * yaw bias + noise, whose integrated error bends the whole trajectory.

Deterministic (fixed seed) so every demo run drifts the same way.

Subscribes:  /odom        (nav_msgs/Odometry, ground truth)
Publishes:   /odom_noisy  (nav_msgs/Odometry, drifting)
"""

from __future__ import annotations

import math
import random

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NoisyOdom(Node):
    def __init__(self) -> None:
        super().__init__("noisy_odom")
        # Systematic scale biases (fraction of motion): 4% forward, 8% lateral
        # (mecanum slip), 5% on rotation. Set all to 0.0 to pass through.
        self.declare_parameter("bias_x", 0.04)
        self.declare_parameter("bias_y", 0.08)
        self.declare_parameter("bias_yaw", 0.05)
        # White noise std-dev per metre travelled / radian turned.
        self.declare_parameter("noise_lin", 0.02)
        self.declare_parameter("noise_yaw", 0.01)
        self.declare_parameter("seed", 7)
        # slam_toolbox needs the classic odom->base_link TF; our homemade
        # slam_node does not (it consumes /odom_noisy directly).
        self.declare_parameter("publish_tf", False)

        self._bx = float(self.get_parameter("bias_x").value)
        self._by = float(self.get_parameter("bias_y").value)
        self._bw = float(self.get_parameter("bias_yaw").value)
        self._nl = float(self.get_parameter("noise_lin").value)
        self._nw = float(self.get_parameter("noise_yaw").value)
        self._rng = random.Random(int(self.get_parameter("seed").value))

        self._last = None            # last ground-truth (x, y, yaw)
        self._pose = None            # integrated noisy (x, y, yaw)
        self._tf = None
        if bool(self.get_parameter("publish_tf").value):
            from tf2_ros import TransformBroadcaster
            self._tf = TransformBroadcaster(self)

        self.pub = self.create_publisher(Odometry, "odom_noisy", 20)
        self.create_subscription(Odometry, "odom", self._on_odom, 20)
        self.get_logger().info(
            "noisy_odom up: /odom -> /odom_noisy "
            f"(bias x/y/yaw = {self._bx:.0%}/{self._by:.0%}/{self._bw:.0%})")

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        gt = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

        if self._pose is None:
            # Start from the true pose (the robot knows where it was switched on).
            self._last = gt
            self._pose = gt
        else:
            # True displacement expressed in the previous BODY frame -- that is
            # what real wheel encoders measure, and where their errors live.
            dxw = gt[0] - self._last[0]
            dyw = gt[1] - self._last[1]
            dyaw = math.atan2(math.sin(gt[2] - self._last[2]),
                              math.cos(gt[2] - self._last[2]))
            c, s = math.cos(-self._last[2]), math.sin(-self._last[2])
            dxb = c * dxw - s * dyw
            dyb = s * dxw + c * dyw
            self._last = gt

            # Corrupt the body-frame increments.
            step = math.hypot(dxb, dyb)
            dxb = dxb * (1.0 + self._bx) + self._rng.gauss(0.0, self._nl * step)
            dyb = dyb * (1.0 + self._by) + self._rng.gauss(0.0, self._nl * step)
            dyaw = dyaw * (1.0 + self._bw) + self._rng.gauss(0.0, self._nw * abs(dyaw))

            # Re-integrate in the drifting frame.
            x, y, yaw = self._pose
            c, s = math.cos(yaw), math.sin(yaw)
            self._pose = (x + c * dxb - s * dyb,
                          y + s * dxb + c * dyb,
                          math.atan2(math.sin(yaw + dyaw), math.cos(yaw + dyaw)))

        out = Odometry()
        out.header = msg.header
        out.header.frame_id = "odom"          # drifting frame, NOT map
        out.child_frame_id = msg.child_frame_id or "base_link"
        out.pose.pose.position.x = self._pose[0]
        out.pose.pose.position.y = self._pose[1]
        out.pose.pose.position.z = p.position.z
        half = self._pose[2] / 2.0
        out.pose.pose.orientation.z = math.sin(half)
        out.pose.pose.orientation.w = math.cos(half)
        out.twist = msg.twist                 # body-frame twist is unaffected
        self.pub.publish(out)

        if self._tf is not None:
            from geometry_msgs.msg import TransformStamped
            t = TransformStamped()
            t.header = out.header
            t.child_frame_id = out.child_frame_id
            t.transform.translation.x = out.pose.pose.position.x
            t.transform.translation.y = out.pose.pose.position.y
            t.transform.rotation = out.pose.pose.orientation
            self._tf.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NoisyOdom()
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
