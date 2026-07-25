"""navigation_node -- pure-pursuit path follower.

Subscribes:
    /plan  (nav_msgs/Path)       waypoints from planning_node
    /odom  (nav_msgs/Odometry)   current pose
Publishes:
    /cmd_vel (geometry_msgs/Twist)  body-frame velocity for the mecanum base

The controller (lib/pure_pursuit.py) is the one validated in Webots. The
Webots driver (youbot_webots) turns /cmd_vel into the four wheel speeds via
the same mecanum kinematics (lib/mecanum.py), so behaviour matches Phase A.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
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
        self.declare_parameter("safety_stop", 0.15)      # m: hardest braking this close
        self.declare_parameter("safety_slow", 0.45)      # m: begin slowing
        self.declare_parameter("safety_halfcone", 0.30)  # rad: forward danger cone
        self.declare_parameter("safety_min", 0.35)       # never brake below this (no freeze)

        self.controller = PurePursuit(
            lookahead=self.get_parameter("lookahead").value,
            cruise_speed=self.get_parameter("cruise_speed").value,
        )
        self._stop = float(self.get_parameter("safety_stop").value)
        self._slow = float(self.get_parameter("safety_slow").value)
        self._halfcone = float(self.get_parameter("safety_halfcone").value)
        self._minfactor = float(self.get_parameter("safety_min").value)
        self._pose = None
        self._scan = None
        self._last_wp = None
        self._reached_logged = False
        self._held = False       # mission owns /cmd_vel during align + pick
        self.create_subscription(Path, "plan", self._on_plan, 1)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "scan", self._on_scan, 5)
        self.create_subscription(Bool, "pick_hold", self._on_hold, 5)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_timer(self.get_parameter("control_period").value, self._control)
        self.get_logger().info("navigation_node up: /plan + /odom (+lidar brake) -> /cmd_vel")

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg

    def _on_hold(self, msg: Bool) -> None:
        self._held = bool(msg.data)

    def _safety_factor(self, direction: float) -> float:
        """Scale [0..1] from the nearest obstacle inside a forward cone around
        `direction` (body frame). 0 = obstacle at safety_stop, 1 = clear."""
        scan = self._scan
        if scan is None or not scan.ranges:
            return 1.0
        nearest = math.inf
        a = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r):
                d = math.atan2(math.sin(a - direction), math.cos(a - direction))
                if abs(d) <= self._halfcone and r < nearest:
                    nearest = r
            a += scan.angle_increment
        if nearest >= self._slow:
            return 1.0
        # Scale down as the obstacle nears, but NEVER to a full stop: a floor of
        # safety_min keeps the base creeping so it clears the point (a gutter
        # end, a wall on a tight turn) instead of freezing there forever.
        scaled = (nearest - self._stop) / (self._slow - self._stop)
        return max(self._minfactor, scaled)

    def _on_plan(self, msg: Path) -> None:
        waypoints = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        if waypoints == self._last_wp:
            return  # identical replan -> don't reset the cursor or re-log
        self._last_wp = waypoints
        self._reached_logged = False
        self.controller.set_path(waypoints)
        self.get_logger().info(f"New plan: {len(waypoints)} waypoints.")

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
        elif status == "running":
            # Brake off the lidar in the direction we're actually moving so the
            # base slows/stops before hitting a gutter or wall, but keeps turning
            # (wz) so it can rotate away and the planner reroutes.
            factor = self._safety_factor(math.atan2(vy, vx))
            self._publish(vx * factor, vy * factor, wz)

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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
