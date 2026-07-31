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
        # Geofence: the base is a kinematic "ghost" (no collision, so physics
        # can never wedge or tip it) -- which also means no wall can stop it.
        # Containment therefore has to be a software protective stop, exactly
        # as on a real robot, where you never let a collision be the limit.
        self.declare_parameter("fence_x", 4.55)          # m: |x| beyond this = out
        self.declare_parameter("fence_y", 2.10)          # m: |y| beyond this = out
        self.declare_parameter("fence_margin", 0.25)     # m: start pushing back here
        # Stuck detector: commanding motion but not moving -> escape manoeuvre.
        self.declare_parameter("stuck_speed", 0.03)      # m/s: below this = not moving
        self.declare_parameter("stuck_time", 4.0)        # s before declaring stuck
        self.declare_parameter("recover_time", 2.0)      # s of escape manoeuvre

        self.controller = PurePursuit(
            lookahead=self.get_parameter("lookahead").value,
            cruise_speed=self.get_parameter("cruise_speed").value,
        )
        self._stop = float(self.get_parameter("safety_stop").value)
        self._slow = float(self.get_parameter("safety_slow").value)
        self._halfcone = float(self.get_parameter("safety_halfcone").value)
        self._minfactor = float(self.get_parameter("safety_min").value)
        self._fx = float(self.get_parameter("fence_x").value)
        self._fy = float(self.get_parameter("fence_y").value)
        self._fmargin = float(self.get_parameter("fence_margin").value)
        self._stuck_speed = float(self.get_parameter("stuck_speed").value)
        self._stuck_time = float(self.get_parameter("stuck_time").value)
        self._recover_time = float(self.get_parameter("recover_time").value)
        self._pose = None
        self._scan = None
        self._last_wp = None
        self._reached_logged = False
        self._held = False       # mission owns /cmd_vel during align + pick
        self._moved_at = None    # last time the base actually moved
        self._last_pos = None
        self._recover_until = 0.0
        self._recover_cmd = (0.0, 0.0, 0.0)
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

    # --- containment and recovery ------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _fence_push(self):
        """Body-frame velocity that pushes the base back inside the arena, or
        None while it is comfortably inside. Beyond the fence this OVERRIDES
        the path follower: no plan is worth leaving the greenhouse for."""
        x, y, yaw = self._pose
        px = py = 0.0
        if x > self._fx - self._fmargin:
            px = -1.0
        elif x < -self._fx + self._fmargin:
            px = 1.0
        if y > self._fy - self._fmargin:
            py = -1.0
        elif y < -self._fy + self._fmargin:
            py = 1.0
        if px == 0.0 and py == 0.0:
            return None
        # Rotate the world-frame push into the body frame.
        c, s = math.cos(-yaw), math.sin(-yaw)
        speed = 0.25
        return (speed * (c * px - s * py), speed * (s * px + c * py), 0.0)

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

        # 1. Escape manoeuvre wins over everything (it is what unblocks us).
        if self._update_stuck((vx, vy)):
            self._publish(*self._recover_cmd)
            return
        # 2. Geofence override: outside the arena, drive back in, nothing else.
        push = self._fence_push()
        if push is not None:
            self._publish(*push)
            return
        # 3. Normal following, braked off the lidar in the direction we are
        # actually moving, but still turning (wz) so it can rotate away.
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
    except (KeyboardInterrupt, RuntimeError):
        pass      # RuntimeError: message mid-shutdown (rclpy)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
