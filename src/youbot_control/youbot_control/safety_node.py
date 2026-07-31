"""safety_node -- the protective stop every /cmd_vel must pass through.

Why a separate node. The base is driven kinematically (Gazebo's
VelocityControl imposes the commanded velocity every physics step), so a
collision can never stop it: the contact response is overwritten by the next
velocity command. Making the walls "solid" therefore cannot work -- on a real
robot you would never let a collision be the limit either. Containment has to
be a protective stop on the COMMAND, and it has to be the last thing between
every publisher and the wheels.

That last point is what the field logs exposed: navigation_node had a lidar
brake, but mission_node takes the base over during alignment and picking and
published straight to /cmd_vel, bypassing it entirely -- which is exactly when
the robot crept sideways into a gutter ("Aligned at (-4.02, +0.84)": in a
0.8 m aisle a 0.38 m-wide base centred at y = 0.84 already overlaps the gutter
edge at y = 1.0). Now EVERY publisher writes /cmd_vel_raw and only this node
writes /cmd_vel, so there is one place where safety is enforced and no way to
route around it.

Three layers, each able to veto:
  1. LIDAR   any beam inside the swept corridor closer than stop_distance
             cancels the translation in that direction (rotation is kept, so
             the robot can always turn away and the planner reroutes).
  2. FENCE   beyond the arena bounds, the only command allowed is the one
             driving back inside.
  3. TIMEOUT no command for cmd_timeout seconds -> stop. A silent publisher
             must never leave the base coasting.

Subscribes:  /cmd_vel_raw (geometry_msgs/Twist)  every publisher
             /scan        (sensor_msgs/LaserScan)
             /odom        (nav_msgs/Odometry)     estimated pose
Publishes:   /cmd_vel     (geometry_msgs/Twist)   the only thing the base sees
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SafetyNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_node")
        self.declare_parameter("stop_distance", 0.30)   # m: hard stop this close
        self.declare_parameter("slow_distance", 0.55)   # m: start scaling down
        self.declare_parameter("half_cone", 0.55)       # rad: swept corridor
        self.declare_parameter("min_valid_range", 0.30)  # m: ignore self-hits
        self.declare_parameter("fence_x", 4.55)
        self.declare_parameter("fence_y", 2.10)
        self.declare_parameter("fence_speed", 0.20)     # m/s pushing back in
        # Lateral containment. Braking only on the COMMANDED direction misses
        # the way the robot actually entered the gutters: during visual
        # alignment the mission commands vx only, and a small heading error
        # turns that into sideways drift nobody was watching. Keeping a
        # minimum clearance on both flanks is what holds it in the aisle --
        # sensor-based, so it works in any corridor, not just this greenhouse.
        self.declare_parameter("side_min", 0.28)        # m: closest flank allowed
        self.declare_parameter("side_push", 0.10)       # m/s correction
        self.declare_parameter("cmd_timeout", 1.0)      # s without a command
        self.declare_parameter("rate", 20.0)

        self._stop = float(self.get_parameter("stop_distance").value)
        self._slow = float(self.get_parameter("slow_distance").value)
        self._cone = float(self.get_parameter("half_cone").value)
        self._min_range = float(self.get_parameter("min_valid_range").value)
        self._fx = float(self.get_parameter("fence_x").value)
        self._fy = float(self.get_parameter("fence_y").value)
        self._fence_speed = float(self.get_parameter("fence_speed").value)
        self._side_min = float(self.get_parameter("side_min").value)
        self._side_push = float(self.get_parameter("side_push").value)
        self._timeout = float(self.get_parameter("cmd_timeout").value)

        self._cmd = None
        self._cmd_t = 0.0
        self._scan = None
        self._pose = None
        self._blocked_since = None

        self.create_subscription(Twist, "cmd_vel_raw", self._on_cmd, 10)
        self.create_subscription(LaserScan, "scan", self._on_scan, 5)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_timer(1.0 / float(self.get_parameter("rate").value), self._tick)
        self.get_logger().info(
            "safety_node up: /cmd_vel_raw -> [lidar stop + arena fence] -> /cmd_vel")

    # ------------------------------------------------------------- inputs
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg: Twist) -> None:
        self._cmd, self._cmd_t = msg, self._now()

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    # ------------------------------------------------------------- layers
    def _clearance(self, direction: float, cone: float = None) -> float:
        """Nearest obstacle inside the corridor swept by moving along
        `direction` (body frame). inf when nothing is in the way."""
        if self._scan is None:
            return float("inf")
        best = float("inf")
        n = len(self._scan.ranges)
        for i, r in enumerate(self._scan.ranges):
            if not math.isfinite(r) or r <= self._min_range or r >= self._scan.range_max:
                continue
            a = self._scan.angle_min + i * self._scan.angle_increment
            da = math.atan2(math.sin(a - direction), math.cos(a - direction))
            if abs(da) <= (self._cone if cone is None else cone) and r < best:
                best = r
        return best

    def _brake(self, vx, vy):
        """Scale translation by proximity in the direction of travel."""
        speed = math.hypot(vx, vy)
        if speed < 1e-4:
            return vx, vy, False
        d = self._clearance(math.atan2(vy, vx))
        if d >= self._slow:
            return vx, vy, False
        if d <= self._stop:
            return 0.0, 0.0, True
        f = (d - self._stop) / (self._slow - self._stop)
        return vx * f, vy * f, False

    def _recentre(self) -> float:
        """Body-frame vy that pushes off whichever flank is too close, 0 when
        both are clear. Cones are narrow (+/-0.35 rad about +/-90 deg) so the
        gutter beside the robot is measured, not the row ahead."""
        left = self._clearance(math.pi / 2.0, cone=0.35)
        right = self._clearance(-math.pi / 2.0, cone=0.35)
        if left < self._side_min and left <= right:
            return -self._side_push          # too close on the left -> go right
        if right < self._side_min:
            return self._side_push
        return 0.0

    def _fence(self):
        """Body-frame command that drives back inside, or None when inside."""
        if self._pose is None:
            return None
        x, y, yaw = self._pose
        px = -1.0 if x > self._fx else (1.0 if x < -self._fx else 0.0)
        py = -1.0 if y > self._fy else (1.0 if y < -self._fy else 0.0)
        if px == 0.0 and py == 0.0:
            return None
        c, s = math.cos(-yaw), math.sin(-yaw)
        v = self._fence_speed
        return (v * (c * px - s * py), v * (s * px + c * py))

    # --------------------------------------------------------------- loop
    def _tick(self) -> None:
        out = Twist()
        # 3. Command timeout -- a silent publisher must not leave us coasting.
        if self._cmd is None or (self._now() - self._cmd_t) > self._timeout:
            self.pub.publish(out)
            return
        vx, vy = self._cmd.linear.x, self._cmd.linear.y
        wz = self._cmd.angular.z

        # 2. Outside the arena: the only allowed motion is back inside.
        push = self._fence()
        if push is not None:
            out.linear.x, out.linear.y = push
            self.pub.publish(out)
            self.get_logger().warn("Outside the arena -- driving back in.",
                                   throttle_duration_sec=3.0)
            return

        # 1. Lidar protective stop (rotation always survives, so the robot can
        # turn away from whatever is blocking it), then flank recentring.
        vx, vy, blocked = self._brake(vx, vy)
        vy += self._recentre()
        out.linear.x, out.linear.y, out.angular.z = vx, vy, wz
        self.pub.publish(out)

        if blocked:
            if self._blocked_since is None:
                self._blocked_since = self._now()
            elif self._now() - self._blocked_since > 3.0:
                self.get_logger().warn(
                    "Obstacle within %.2f m -- translation held." % self._stop,
                    throttle_duration_sec=5.0)
        else:
            self._blocked_since = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
