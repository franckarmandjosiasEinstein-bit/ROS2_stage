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
  1. LIDAR   anything inside the RECTANGLE the robot will sweep along the
             commanded direction cancels the translation (rotation is kept, so
             the robot can always turn away and the planner reroutes).
  2. FENCE   beyond the arena bounds, the only command allowed is the one
             driving back inside.
  3. TIMEOUT no command for cmd_timeout seconds -> stop. A silent publisher
             must never leave the base coasting.

Why a swept rectangle and not a cone. The first version tested a +/-0.55 rad
cone around the direction of travel and took the raw beam range. Two things go
wrong with that, and together they pinned the robot in the top margin for a
whole run. A cone widens with distance, so at 0.5 m it already reaches 0.28 m
sideways: a beam grazing the gutter the robot is legitimately driving PAST
comes back short and brakes it, in open corridor. And the range along an
oblique beam is not the distance to the obstacle along the path -- it
overstates how soon we arrive. The rectangle asks the only question that
matters: projected on the direction of travel, how far ahead is the nearest
point that lies within the robot's own width?

Subscribes:  /cmd_vel_raw (geometry_msgs/Twist)  every publisher
             /scan        (sensor_msgs/LaserScan)
             /odom        (nav_msgs/Odometry)     estimated pose
Publishes:   /cmd_vel     (geometry_msgs/Twist)   the only thing the base sees
             /safety_override (std_msgs/Bool)     true while the guard is
                 overriding rather than passing the command through. A
                 publisher that keeps commanding into an override just fights
                 it: the field log shows twenty seconds of "Outside the arena"
                 every three seconds while the mission kept creeping sideways
                 after a berry. Whoever is driving can now see it and stop.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SafetyNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_node")
        self.declare_parameter("stop_distance", 0.28)   # m: hard stop this close
        self.declare_parameter("slow_distance", 0.60)   # m: start scaling down
        # Half-width of the swept corridor: the base is 0.38 m wide (0.19 half)
        # and the wheels stand a little proud, so 0.24 covers the body with a
        # small margin. Wider than that and the robot brakes for walls it is
        # merely driving alongside.
        self.declare_parameter("half_width", 0.24)
        self.declare_parameter("min_valid_range", 0.30)  # m: ignore self-hits
        # Arena bounds, as the walls the BODY must stay inside -- not the
        # centre. The greenhouse is 10 x 5 m with 0.10 m walls, so the inner
        # faces are at +/-4.95 and +/-2.45; the fence checks all four corners
        # of the 0.58 x 0.38 m footprint against them, rotated by the current
        # heading. Bounding the centre alone is what let the chassis clip the
        # glass on a diagonal: at 45 deg the corner reaches 0.35 m out, not
        # the 0.19 m a side-on estimate suggests.
        self.declare_parameter("fence_x", 4.85)
        self.declare_parameter("fence_y", 2.35)
        self.declare_parameter("base_length", 0.58)
        self.declare_parameter("base_width", 0.38)
        self.declare_parameter("fence_speed", 0.20)     # m/s pushing back in
        # Lateral containment. Braking only on the COMMANDED direction misses
        # the way the robot actually entered the gutters: during visual
        # alignment the mission commands vx only, and a small heading error
        # turns that into sideways drift nobody was watching. Keeping a
        # minimum clearance on both flanks is what holds it in the aisle --
        # sensor-based, so it works in any corridor, not just this greenhouse.
        self.declare_parameter("side_min", 0.30)        # m: closest flank allowed
        self.declare_parameter("side_push", 0.12)       # m/s max correction
        self.declare_parameter("side_band", 0.35)       # m: fore/aft extent measured
        self.declare_parameter("cmd_timeout", 1.0)      # s without a command
        self.declare_parameter("rate", 20.0)

        self._stop = float(self.get_parameter("stop_distance").value)
        self._slow = float(self.get_parameter("slow_distance").value)
        self._half_w = float(self.get_parameter("half_width").value)
        self._min_range = float(self.get_parameter("min_valid_range").value)
        self._fx = float(self.get_parameter("fence_x").value)
        self._fy = float(self.get_parameter("fence_y").value)
        self._hl = float(self.get_parameter("base_length").value) / 2.0
        self._hw = float(self.get_parameter("base_width").value) / 2.0
        self._fence_speed = float(self.get_parameter("fence_speed").value)
        self._side_min = float(self.get_parameter("side_min").value)
        self._side_push = float(self.get_parameter("side_push").value)
        self._side_band = float(self.get_parameter("side_band").value)
        self._timeout = float(self.get_parameter("cmd_timeout").value)

        self._cmd = None
        self._cmd_t = 0.0
        self._pts = []           # body-frame (x, y) of every valid return
        self._pose = None
        self._blocked_since = None

        self.create_subscription(Twist, "cmd_vel_raw", self._on_cmd, 10)
        self.create_subscription(LaserScan, "scan", self._on_scan, 5)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.override_pub = self.create_publisher(Bool, "safety_override", 5)
        self.create_timer(1.0 / float(self.get_parameter("rate").value), self._tick)
        self.get_logger().info(
            "safety_node up: /cmd_vel_raw -> [swept-corridor stop + flank "
            "recentring + arena fence] -> /cmd_vel")

    # ------------------------------------------------------------- inputs
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg: Twist) -> None:
        self._cmd, self._cmd_t = msg, self._now()

    def _on_scan(self, msg: LaserScan) -> None:
        """Cache the scan once as body-frame points. Every layer below is a
        geometric test on those points, so the trigonometry is done once per
        scan instead of once per query."""
        pts = []
        a = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and self._min_range < r < msg.range_max:
                pts.append((r * math.cos(a), r * math.sin(a)))
            a += msg.angle_increment
        self._pts = pts

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    # ------------------------------------------------------------- layers
    def _corridor_clearance(self, direction: float) -> float:
        """Distance ALONG `direction` (body frame) to the nearest return that
        falls inside the rectangle the base will sweep going that way.
        inf when the corridor is clear."""
        ux, uy = math.cos(direction), math.sin(direction)
        best = float("inf")
        for px, py in self._pts:
            along = px * ux + py * uy
            if along <= 0.0 or along >= best:
                continue
            if abs(-px * uy + py * ux) <= self._half_w:   # inside our width
                best = along
        return best

    def _brake(self, vx, vy):
        """Scale translation by how far the swept corridor is clear."""
        speed = math.hypot(vx, vy)
        if speed < 1e-4:
            return vx, vy, False
        d = self._corridor_clearance(math.atan2(vy, vx))
        if d >= self._slow:
            return vx, vy, False
        if d <= self._stop:
            return 0.0, 0.0, True
        f = (d - self._stop) / (self._slow - self._stop)
        return vx * f, vy * f, False

    def _flank(self, sign: float) -> float:
        """Perpendicular clearance on one side (+1 left, -1 right), measured
        over a band level with the base rather than a cone -- a cone at 90 deg
        picks up whatever is diagonally ahead and calls it a wall."""
        best = float("inf")
        for px, py in self._pts:
            side = sign * py
            if 0.0 < side < best and abs(px) <= self._side_band:
                best = side
        return best

    def _recentre(self, translating: bool) -> float:
        """Body-frame vy that pushes off whichever flank is too close.

        Two guards, both learned from the run where the robot spent a whole
        lap trapped in the top margin. It only acts while the robot is
        TRANSLATING -- injecting sideways velocity into a stationary or purely
        rotating base fights the mission's own alignment. And it only pushes
        toward a side that actually has room: in a passage narrower than
        2*side_min every correction is a shove into the opposite wall, so it
        holds still and lets the brake and the planner deal with it."""
        if not translating:
            return 0.0
        left, right = self._flank(1.0), self._flank(-1.0)
        room = self._side_min + 0.08
        if left < self._side_min and right > room:
            return -self._side_push * min(1.0, (self._side_min - left) / self._side_min)
        if right < self._side_min and left > room:
            return self._side_push * min(1.0, (self._side_min - right) / self._side_min)
        return 0.0

    def _overhang(self):
        """How far the footprint pokes past each wall, as (dx, dy) in world
        metres. (0, 0) while the whole body is inside."""
        x, y, yaw = self._pose
        c, s = math.cos(yaw), math.sin(yaw)
        ox = oy = 0.0
        for sl, sw in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
            cx = x + sl * self._hl * c - sw * self._hw * s
            cy = y + sl * self._hl * s + sw * self._hw * c
            if cx > self._fx:
                ox = min(ox, self._fx - cx)
            elif cx < -self._fx:
                ox = max(ox, -self._fx - cx)
            if cy > self._fy:
                oy = min(oy, self._fy - cy)
            elif cy < -self._fy:
                oy = max(oy, -self._fy - cy)
        return ox, oy

    def _fence(self):
        """Body-frame command that drives back inside, or None when inside.

        No soft margin on purpose. navigation_node used to run its own fence
        with a 0.25 m margin while this one had none, and the two disagreed
        about where the boundary was: the robot ping-ponged along x = -4.30
        for an entire run. One fence, one boundary, and it only speaks when
        part of the body is genuinely out."""
        if self._pose is None:
            return None
        ox, oy = self._overhang()
        if ox == 0.0 and oy == 0.0:
            return None
        # Push proportionally to the overhang, so a corner 2 cm past the glass
        # gets a nudge and not the full 0.20 m/s lurch.
        n = math.hypot(ox, oy)
        gain = min(1.0, n / 0.10)
        yaw = self._pose[2]
        c, s = math.cos(-yaw), math.sin(-yaw)
        v = self._fence_speed * gain / n
        return (v * (c * ox - s * oy), v * (s * ox + c * oy))

    # --------------------------------------------------------------- loop
    def _tick(self) -> None:
        out = Twist()
        # 3. Command timeout -- a silent publisher must not leave us coasting.
        if self._cmd is None or (self._now() - self._cmd_t) > self._timeout:
            self.pub.publish(out)
            self.override_pub.publish(Bool(data=False))
            return
        vx, vy = self._cmd.linear.x, self._cmd.linear.y
        wz = self._cmd.angular.z

        # 2. Outside the arena: the only allowed motion is back inside.
        push = self._fence()
        if push is not None:
            out.linear.x, out.linear.y = push
            self.pub.publish(out)
            self.override_pub.publish(Bool(data=True))
            self.get_logger().warn("Outside the arena -- driving back in.",
                                   throttle_duration_sec=3.0)
            return

        # 1. Swept-corridor protective stop (rotation always survives, so the
        # robot can turn away from whatever is blocking it), then recentring.
        # Deliberately low: the mission's visual alignment creeps at a few
        # cm/s, and that slow creep is precisely how the base used to end up
        # overlapping a gutter. Anything that moves gets flank protection.
        translating = math.hypot(vx, vy) > 0.02
        vx, vy, blocked = self._brake(vx, vy)
        vy += self._recentre(translating)
        out.linear.x, out.linear.y, out.angular.z = vx, vy, wz
        self.pub.publish(out)
        # NOT an override. The brake scales the commanded direction down; the
        # commander is still in control and closing on its target. Only the
        # fence REPLACES the command, and only that is worth telling anyone
        # about: publishing the brake here made mission_node throw away a
        # berry sitting at offset +0.01 -- dead centre -- because standing
        # next to a gutter naturally trips the brake.
        self.override_pub.publish(Bool(data=False))

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
