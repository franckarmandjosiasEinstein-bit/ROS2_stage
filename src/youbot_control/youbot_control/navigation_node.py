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
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from youbot_control.lib.clearance import best_escape
from youbot_control.lib.pure_pursuit import PurePursuit


def _wrap(a: float) -> float:
    """Shortest signed angle. A yaw crossing +/-pi between two ticks is a
    0.001 rad turn, not a 6.28 rad one -- and the stuck detector reads it."""
    return math.atan2(math.sin(a), math.cos(a))


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
        # And the same test on the HEADING. The stuck detector used to ask
        # only "am I commanding translation and not translating?", which has a
        # blind spot the size of the whole failure: pure pursuit refuses to
        # translate while the heading error exceeds 90 deg, so at the end of
        # an aisle the command is a PURE ROTATION. If the guard then cancels
        # that rotation -- which it does when turning would sweep a corner
        # through the fence -- the robot is commanding motion, achieving none,
        # and reporting nothing, because `want` was false and _moved_at was
        # reset every single tick. That is the ten-minute freeze at
        # (-4.46, 1.49, 101 deg), and this parameter is what notices it.
        self.declare_parameter("stuck_yaw", 0.10)        # rad/s: below this = not turning
        self.declare_parameter("stuck_time", 4.0)        # s before declaring stuck
        self.declare_parameter("recover_time", 2.0)      # s of escape manoeuvre
        self.declare_parameter("escape_speed", 0.18)     # m/s during the escape
        # Footprint, so the escape scores directions the same way the guard
        # brakes them (lib/clearance.py). A recovery that reverses into a
        # direction safety_node then cancels is the loop the field log shows:
        # "Stuck at (+3.70, +0.61)" ten times in nine minutes, same spot.
        self.declare_parameter("corridor_margin", 0.02)
        self.declare_parameter("base_length", 0.58)
        self.declare_parameter("base_width", 0.38)
        self.declare_parameter("min_valid_range", 0.30)
        # How stale a pose may be before the follower refuses to steer on it.
        # One second is twenty control periods: long enough to ride out a
        # dropped message, short enough that a dead pose source is named
        # within a second instead of after twelve minutes.
        self.declare_parameter("pose_timeout", 1.0)       # s
        # Repeated stucks within this radius = the same trap, not bad luck.
        self.declare_parameter("trap_radius", 0.40)      # m
        self.declare_parameter("trap_count", 3)          # then give the goal up

        self.controller = PurePursuit(
            lookahead=self.get_parameter("lookahead").value,
            cruise_speed=self.get_parameter("cruise_speed").value,
        )
        self._stuck_speed = float(self.get_parameter("stuck_speed").value)
        self._stuck_yaw = float(self.get_parameter("stuck_yaw").value)
        self._stuck_time = float(self.get_parameter("stuck_time").value)
        self._recover_time = float(self.get_parameter("recover_time").value)
        self._escape_speed = float(self.get_parameter("escape_speed").value)
        self._margin = float(self.get_parameter("corridor_margin").value)
        self._hl = float(self.get_parameter("base_length").value) / 2.0
        self._hw = float(self.get_parameter("base_width").value) / 2.0
        self._min_range = float(self.get_parameter("min_valid_range").value)
        self._pose_timeout = float(self.get_parameter("pose_timeout").value)
        self._trap_r = float(self.get_parameter("trap_radius").value)
        self._trap_n = int(self.get_parameter("trap_count").value)
        self._pose = None
        # When the last pose arrived. A pose that STOPS ARRIVING is invisible
        # otherwise: self._pose keeps its final value, the follower keeps
        # steering toward a lookahead point from a position the robot left
        # long ago, and nothing in the log says why the robot is not moving.
        # That is exactly what a run looked like -- the truth pose frozen at
        # (-4.45, 1.86) for twelve minutes while the mission cheerfully issued
        # goals and the planner cheerfully planned to them.
        self._pose_t = None
        self._pose_lost = False
        self._last_wp = None
        self._reached_logged = False
        self._held = False       # mission owns /cmd_vel during align + pick
        self._held_since = 0.0   # when the hold started (freeze diagnostic)
        self._moved_at = None    # last time the base actually moved OR turned
        self._last_pos = None    # (x, y, yaw) at the previous tick
        self._recover_until = 0.0
        self._recover_cmd = (0.0, 0.0, 0.0)
        self._pts = []            # body-frame lidar returns (escape direction)
        self._trap_xy = None      # where the last stuck happened
        self._trap_hits = 0       # consecutive stucks in the same place
        self.create_subscription(Path, "plan", self._on_plan, 1)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(Bool, "pick_hold", self._on_hold, 5)
        self.create_subscription(LaserScan, "scan", self._on_scan,
                                 qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # "This goal is not reachable from here." mission_node abandons on it
        # immediately instead of burning its 60 s timeout twice over, which is
        # what the field log spent nine minutes doing.
        self.blocked_pub = self.create_publisher(Bool, "goal_blocked", 5)
        # "I have driven the whole path." Distinct from goal_blocked, which
        # means "I cannot move at all": the path can complete perfectly and
        # still stop short of the goal the mission asked for.
        self.done_pub = self.create_publisher(Bool, "path_done", 5)
        self.create_timer(self.get_parameter("control_period").value, self._control)
        self.get_logger().info(
            "navigation_node up: /plan + /odom -> /cmd_vel_raw (safety_node guards)")

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        self._pose_t = self._now()

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
            if math.hypot(commanded[0], commanded[1]) > 1e-6 else None
        d, gap = best_escape(self._pts, self._margin, self._hl, self._hw,
                             avoid=want)
        if d is None or gap < 0.05:
            # Boxed in on every heading (or no scan yet): rotate in place and
            # hand the planner a new view. This is NOT unconditionally allowed
            # -- safety_node's preventive fence scales rotation back too when
            # turning would sweep a corner out -- which is why the strafe
            # below is the primary escape and this is the fallback, not the
            # other way round.
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
        """True while an escape manoeuvre is running.

        A robot that commands motion but does not move is jammed (a corner, a
        map artefact, or the guard refusing the command). `commanded` is the
        full body twist (vx, vy, wz), and BOTH halves count: a cancelled
        rotation is as much a jam as a cancelled translation, and it is the
        one that goes unnoticed, because a robot that is only trying to turn
        never trips a translation test. See the `stuck_yaw` parameter."""
        t = self._now()
        if t < self._recover_until:
            # Re-pick the direction on every tick rather than replaying the
            # command frozen at the moment we noticed. The robot moves during
            # the escape, so a command computed once goes stale and can end up
            # driving into whatever has come round in front of it.
            vx, vy, wz, _, _ = self._escape(commanded)
            self._recover_cmd = (vx, vy, wz)
            return True
        x, y, yaw = self._pose
        if self._last_pos is None:
            self._last_pos, self._moved_at = (x, y, yaw), t
            return False
        moved = math.hypot(x - self._last_pos[0], y - self._last_pos[1])
        turned = abs(_wrap(yaw - self._last_pos[2]))
        self._last_pos = (x, y, yaw)
        want = (math.hypot(commanded[0], commanded[1]) > self._stuck_speed
                or abs(commanded[2]) > self._stuck_yaw)
        progress = (moved > self._stuck_speed * 0.05
                    or turned > self._stuck_yaw * 0.05)
        if not want or progress:
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
        # Name WHICH half was refused. "Not turning" points at the fence and
        # the heading; "not moving" points at an obstacle. Two different
        # faults that used to print the same line.
        kind = ("commanded %+.2f rad/s and did not turn"
                % commanded[2]) \
            if math.hypot(commanded[0], commanded[1]) <= self._stuck_speed \
            else ("commanded %.2f m/s and did not move"
                  % math.hypot(commanded[0], commanded[1]))
        self.get_logger().warn(
            f"Stuck at ({x:+.2f}, {y:+.2f}, {math.degrees(yaw):+.0f} deg) "
            f"[{self._trap_hits}] -- {kind} -- {where}.")
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
        if self._pose is None:
            self.get_logger().warn(
                "No pose has EVER arrived -- nothing can drive. Check the "
                "pose source is publishing (slam_node / odom_tf).",
                throttle_duration_sec=10.0)
            self._publish(0.0, 0.0, 0.0)
            return
        # A pose that stopped arriving is worse than no pose at all: every
        # consumer keeps working on a position the robot has left, and the
        # only symptom is a robot that does not move. Refuse to steer on a
        # stale pose, and NAME the fault.
        age = self._now() - self._pose_t
        if age > self._pose_timeout:
            if not self._pose_lost:
                self._pose_lost = True
                self.get_logger().error(
                    f"POSE STALE: nothing has arrived for {age:.1f} s. The "
                    "robot is stopped on purpose -- steering on a stale pose "
                    "drives blind. The pose source (slam_node) has gone "
                    "silent; the mission and the planner do NOT know this and "
                    "will keep issuing goals.")
            else:
                self.get_logger().error(
                    f"POSE STALE for {age:.0f} s -- still stopped.",
                    throttle_duration_sec=15.0)
            self._publish(0.0, 0.0, 0.0)
            return
        if self._pose_lost:
            self._pose_lost = False
            self.get_logger().info("Pose is flowing again -- resuming.")
        if self.controller.is_finished():
            self._publish(0.0, 0.0, 0.0)
            return
        status, vx, vy, wz = self.controller.step(*self._pose)
        if status == "success":
            self._publish(0.0, 0.0, 0.0)
            if not self._reached_logged:
                self._reached_logged = True
                self.get_logger().info("Goal reached.")
                # And SAY SO. Finishing the path used to be a private fact:
                # the follower stopped, and the mission -- which has its own,
                # stricter idea of arrival -- kept waiting for a gap that
                # would never close, burned its 60 s timeout, and re-issued
                # the same goal. The plan legitimately ends short when the
                # goal sits inside inflated space: half a grid cell plus the
                # footprint inflation is up to ~0.4 m, well outside the
                # mission's 0.30 m tolerance. Neither party was wrong; they
                # simply never spoke to each other.
                self.done_pub.publish(Bool(data=True))
            return
        if status != "running":
            return

        # The escape manoeuvre wins over the path: it is what unblocks us.
        # It still goes through safety_node like every other command -- if
        # reversing is what is blocked, the rotation survives and the robot
        # turns its way out instead.
        if self._update_stuck((vx, vy, wz)):
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
