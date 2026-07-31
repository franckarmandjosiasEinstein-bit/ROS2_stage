"""navigation_node -- pure-pursuit path follower.

Subscribes:
    /plan  (nav_msgs/Path)       waypoints from planning_node
    /odom  (nav_msgs/Odometry)   current pose
Publishes:
    /cmd_vel (geometry_msgs/Twist)  body-frame velocity for the mecanum base
        (remapped to /cmd_vel_raw by the launch files: safety_node is the only
        node that writes the real /cmd_vel)

The controller (lib/pure_pursuit.py) is the one validated in Webots. The
Webots driver (youbot_webots) turns /cmd_vel into the four wheel speeds via
the same mecanum kinematics (lib/mecanum.py), so behaviour matches Phase A.

Obstacle braking and the arena fence used to live here as well. They were
moved out to safety_node once mission_node started driving the base too --
two nodes enforcing containment with different numbers is worse than one,
and the disagreement was measurable: this node pushed back inside from
0.25 m before the boundary while safety_node only reacted past it, and the
robot ping-ponged along x = -4.30 for an entire run instead of harvesting.
What stays here is the stuck detector, because only the follower knows the
difference between "asked to move and didn't" and "deliberately stopped".
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool

from youbot_control.lib.pure_pursuit import PurePursuit


def yaw_from_quaternion(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class NavigationNode(Node):
    def __init__(self) -> None:
        super().__init__("navigation_node")
        self.declare_parameter("lookahead", 0.4)
        self.declare_parameter("cruise_speed", 0.5)
        self.declare_parameter("control_period", 0.05)  # 20 Hz
        # Stuck detector: commanding motion but not moving -> escape manoeuvre.
        # (Braking and the arena fence are safety_node's job -- see the module
        # docstring. This node commands the path it wants; the guard decides
        # what actually reaches the wheels.)
        self.declare_parameter("stuck_speed", 0.03)      # m/s: below this = not moving
        self.declare_parameter("stuck_time", 4.0)        # s before declaring stuck
        self.declare_parameter("recover_time", 2.0)      # s of escape manoeuvre

        self.controller = PurePursuit(
            lookahead=self.get_parameter("lookahead").value,
            cruise_speed=self.get_parameter("cruise_speed").value,
        )
        self._stuck_speed = float(self.get_parameter("stuck_speed").value)
        self._stuck_time = float(self.get_parameter("stuck_time").value)
        self._recover_time = float(self.get_parameter("recover_time").value)
        self._pose = None
        self._last_wp = None
        self._reached_logged = False
        self._held = False       # mission owns /cmd_vel during align + pick
        self._moved_at = None    # last time the base actually moved
        self._last_pos = None
        self._recover_until = 0.0
        self._recover_cmd = (0.0, 0.0, 0.0)
        self.create_subscription(Path, "plan", self._on_plan, 1)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(Bool, "pick_hold", self._on_hold, 5)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_timer(self.get_parameter("control_period").value, self._control)
        self.get_logger().info(
            "navigation_node up: /plan + /odom -> /cmd_vel_raw (safety_node guards)")

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def _on_hold(self, msg: Bool) -> None:
        self._held = bool(msg.data)

    def _on_plan(self, msg: Path) -> None:
        waypoints = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        if waypoints == self._last_wp:
            return  # identical replan -> don't reset the cursor or re-log
        self._last_wp = waypoints
        self._reached_logged = False
        self.controller.set_path(waypoints)
        self.get_logger().info(f"New plan: {len(waypoints)} waypoints.")

    # --- recovery ------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _update_stuck(self, commanded) -> bool:
        """True while an escape manoeuvre is running. A robot that commands
        motion but does not move is jammed (a corner, a map artefact): back
        off along its own heading and rotate so the planner gets a new view."""
        t = self._now()
        if t < self._recover_until:
            return True
        x, y, _ = self._pose
        if self._last_pos is None:
            self._last_pos, self._moved_at = (x, y), t
            return False
        moved = math.hypot(x - self._last_pos[0], y - self._last_pos[1])
        self._last_pos = (x, y)
        want = math.hypot(commanded[0], commanded[1]) > self._stuck_speed
        if not want or moved > self._stuck_speed * 0.05:
            self._moved_at = t
            return False
        if t - self._moved_at < self._stuck_time:
            return False
        self._recover_until = t + self._recover_time
        self._recover_cmd = (-0.15, 0.0, 0.6)      # reverse + turn away
        self._moved_at = t
        self.get_logger().warn(
            f"Stuck at ({x:+.2f}, {y:+.2f}) -- backing off and turning.")
        return True

    def _control(self) -> None:
        if self._held:
            return  # mission drives the base directly (aligning / picking)
        if self._pose is None or self.controller.is_finished():
            self._publish(0.0, 0.0, 0.0)
            return
        status, vx, vy, wz = self.controller.step(*self._pose)
        if status == "success":
            self._publish(0.0, 0.0, 0.0)
            if not self._reached_logged:
                self._reached_logged = True
                self.get_logger().info("Goal reached.")
            return
        if status != "running":
            return

        # The escape manoeuvre wins over the path: it is what unblocks us.
        # It still goes through safety_node like every other command -- if
        # reversing is what is blocked, the rotation survives and the robot
        # turns its way out instead.
        if self._update_stuck((vx, vy)):
            self._publish(*self._recover_cmd)
            return
        self._publish(vx, vy, wz)

    def _publish(self, vx, vy, wz) -> None:
        t = Twist()
        t.linear.x = float(vx)
        t.linear.y = float(vy)
        t.angular.z = float(wz)
        self.cmd_pub.publish(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationNode()
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
