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
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from youbot_control.lib.clearance import best_escape
from youbot_control.lib.pure_pursuit import PurePursuit


def yaw_from_quaternion(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class NavigationNode(Node):
    def __init__(self) -> None:
        super().__init__("navigation_node")
        self.declare_parameter("lookahead", 0.32)
        self.declare_parameter("cruise_speed", 0.6)
        self.declare_parameter("control_period", 0.05)  # 20 Hz
        # Stuck detector: commanding motion but not moving -> escape manoeuvre.
        # (Braking and the arena fence are safety_node's job -- see the module
        # docstring. This node commands the path it wants; the guard decides
        # what actually reaches the wheels.)
        self.declare_parameter("stuck_speed", 0.03)      # m/s: below this = not moving
        self.declare_parameter("stuck_time", 4.0)        # s before declaring stuck
        self.declare_parameter("recover_time", 2.0)      # s of escape manoeuvre
        self.declare_parameter("escape_speed", 0.18)     # m/s during the escape
        # Footprint, so the escape scores directions the same way the guard
        # brakes them (lib/clearance.py). A recovery that reverses into a
        # direction safety_node then cancels is the loop the field log shows:
        # "Stuck at (+3.70, +0.61)" ten times in nine minutes, same spot.
        self.declare_parameter("half_width", 0.24)
        self.declare_parameter("base_length", 0.58)
        self.declare_parameter("base_width", 0.38)
        self.declare_parameter("min_valid_range", 0.30)
        # Repeated stucks within this radius = the same trap, not bad luck.
        self.declare_parameter("trap_radius", 0.40)      # m
        self.declare_parameter("trap_count", 3)          # then give the goal up

        self.controller = PurePursuit(
            lookahead=self.get_parameter("lookahead").value,
            cruise_speed=self.get_parameter("cruise_speed").value,
        )
        self._stuck_speed = float(self.get_parameter("stuck_speed").value)
        self._stuck_time = float(self.get_parameter("stuck_time").value)
        self._recover_time = float(self.get_parameter("recover_time").value)
        self._escape_speed = float(self.get_parameter("escape_speed").value)
        self._half_w = float(self.get_parameter("half_width").value)
        self._hl = float(self.get_parameter("base_length").value) / 2.0
        self._hw = float(self.get_parameter("base_width").value) / 2.0
        self._min_range = float(self.get_parameter("min_valid_range").value)
        self._trap_r = float(self.get_parameter("trap_radius").value)
        self._trap_n = int(self.get_parameter("trap_count").value)
        self._pose = None
        self._last_wp = None
        self._reached_logged = False
        self._held = False       # mission owns /cmd_vel during align + pick
        self._held_since = 0.0   # when the hold started (freeze diagnostic)
        self._moved_at = None    # last time the base actually moved
        self._last_pos = None
        self._recover_until = 0.0
        self._recover_cmd = (0.0, 0.0, 0.0)
        self._pts = []            # body-frame lidar returns (escape direction)
        self._trap_xy = None      # where the last stuck happened
        self._trap_hits = 0       # consecutive stucks in the same place
        self.create_subscription(Path, "plan", self._on_plan, 1)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(Bool, "pick_hold", self._on_hold, 5)
        self.create_subscription(LaserScan, "scan", self._on_scan, 5)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # "This goal is not reachable from here." mission_node abandons on it
        # immediately instead of burning its 60 s timeout twice over, which is
        # what the field log spent nine minutes doing.
        self.blocked_pub = self.create_publisher(Bool, "goal_blocked", 5)
        self.create_timer(self.get_parameter("control_period").value, self._control)
        self.get_logger().info(
            "navigation_node up: /plan + /odom -> /cmd_vel_raw (safety_node guards)")

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def _on_scan(self, msg: LaserScan) -> None:
        pts = []
        a = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and self._min_range < r < msg.range_max:
                pts.append((r * math.cos(a), r * math.sin(a)))
            a += msg.angle_increment
        self._pts = pts

    def _on_hold(self, msg: Bool) -> None:
        held = bool(msg.data)
        if held and not self._held:
            self._held_since = self._now()
        self._held = held

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

    def _escape(self, commanded):
        """Body-frame (vx, vy, wz) that drives out of wherever we are wedged.

        The old manoeuvre was the constant (-0.15, 0, 0.6): reverse along the
        current heading and spin. In the greenhouse that is often the one
        direction with no room -- at the end of an aisle the way out is
        sideways -- so it fired, achieved nothing, and fired again ten times
        over nine minutes at the same coordinates.

        Now the lidar picks the direction: the free space in front of the
        FOOTPRINT is scored all the way around (lib/clearance.py, the same
        function safety_node brakes with, so the guard cannot veto the escape
        it just chose), headings near the one already failing are excluded,
        and the base strafes toward the winner while turning to face it. The
        mecanum base can do that without turning first, which is exactly what
        it is for.
        """
        want = math.atan2(commanded[1], commanded[0]) \
            if math.hypot(*commanded) > 1e-6 else None
        d, gap = best_escape(self._pts, self._half_w, self._hl, self._hw,
                             avoid=want)
        if d is None or gap < 0.05:
            # Boxed in on every heading (or no scan yet): rotate in place. It
            # is always allowed -- safety_node only ever vetoes translation --
            # and it hands the planner a new view.
            return 0.0, 0.0, 0.6, d, gap
        # STRAFE ONLY. The first version also turned toward the opening at up
        # to 0.6 rad/s and held that for the whole recovery: two to six seconds
        # of spin, 70 to 200 degrees. The robot came out of every escape
        # crossways in its aisle -- the field log shows it stuck at yaw 17, 29,
        # 76, 128, 147 degrees in a corridor that runs due east -- so the
        # swept corridor then pointed at a gutter, the guard braked, and the
        # next escape turned it further. 58% of the run blocked. A mecanum base
        # does not need to face where it is going; leave the heading alone and
        # let pure pursuit re-aim once there is room.
        v = self._escape_speed
        return v * math.cos(d), v * math.sin(d), 0.0, d, gap

    def _update_stuck(self, commanded) -> bool:
        """True while an escape manoeuvre is running. A robot that commands
        motion but does not move is jammed (a corner, a map artefact)."""
        t = self._now()
        if t < self._recover_until:
            # Re-pick the direction on every tick rather than replaying the
            # command frozen at the moment we noticed. The robot moves during
            # the escape, so a command computed once goes stale and can end up
            # driving into whatever has come round in front of it.
            vx, vy, wz, _, _ = self._escape(commanded)
            self._recover_cmd = (vx, vy, wz)
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

        # Same place as last time? Then the escape is not working and no
        # amount of repeating it will help -- this goal is unreachable from
        # here. Tell the mission so it advances now rather than at the 60 s
        # timeout, twice.
        if self._trap_xy is not None and \
                math.hypot(x - self._trap_xy[0], y - self._trap_xy[1]) < self._trap_r:
            self._trap_hits += 1
        else:
            self._trap_hits = 1
        self._trap_xy = (x, y)

        vx, vy, wz, d, gap = self._escape(commanded)
        # Escalate: a trap that survived one escape gets a longer one.
        self._recover_until = t + self._recover_time * min(3, self._trap_hits)
        self._recover_cmd = (vx, vy, wz)
        self._moved_at = t
        where = "no opening" if d is None else \
            f"escaping toward {math.degrees(d):+.0f} deg ({gap:.2f} m free)"
        self.get_logger().warn(
            f"Stuck at ({x:+.2f}, {y:+.2f}) [{self._trap_hits}] -- {where}.")
        if self._trap_hits >= self._trap_n:
            self.get_logger().warn(
                f"Stuck {self._trap_hits} times inside {self._trap_r:.2f} m "
                "-- reporting the goal as blocked.")
            self.blocked_pub.publish(Bool(data=True))
            self._trap_hits = 0
            self._trap_xy = None
        return True

    def _control(self) -> None:
        if self._held:
            # The mission owns the base (aligning / picking). It hands it back
            # with pick_hold=False; if it ever dies mid-pick it never will, and
            # this node then publishes NOTHING for the rest of the run -- the
            # guard times out and the robot is frozen with no message anywhere
            # explaining why. Say so rather than going quiet.
            if self._now() - self._held_since > 60.0:
                self.get_logger().warn(
                    "Base held by mission_node for over 60 s (pick_hold=true) "
                    "-- nothing is driving. Check mission_node is alive.",
                    throttle_duration_sec=20.0)
            return
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
