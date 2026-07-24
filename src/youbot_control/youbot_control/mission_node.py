"""mission_node -- high-level orchestrator (patrol -> discover -> collect).

Sensor-driven mission: the robot patrols coverage waypoints while a perception
source (the real vision_node in Webots, or sim_node's FOV cone headless)
publishes /detected_crates. Newly seen crates are remembered; once the patrol
is done the robot drives to each, "picks" it (a TODO placeholder for the arm
action), and finally delivers at the depot.

Subscribes:  /odom (nav_msgs/Odometry), /detected_crates (geometry_msgs/PoseArray)
Publishes:   /goal_pose (geometry_msgs/PoseStamped)
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry

# Greenhouse coverage patrol: sweep the two 0.8 m aisles (Y = +/-0.6) and the
# side margins (Y = +/-1.9) so every gutter comes into the camera's view.
PATROL = [
    (-3.8, 1.9), (3.8, 1.9), (3.8, 0.6), (-3.8, 0.6),
    (-3.8, -0.6), (3.8, -0.6), (3.8, -1.9), (-3.8, -1.9),
]
DEPOT = (4.6, 0.0)         # right cross-corridor
ARRIVAL_TOLERANCE = 0.30   # m
DEDUP_DIST = 0.6           # m: merge detections into one crate
GOAL_TIMEOUT = 60.0        # s: abandon a goal we can't reach, advance to next
                           # (long enough for the far depot diagonal)


class MissionNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_node")
        self._pose = None
        self._known = []        # confirmed crate positions (x, y)
        self._collected = []    # crates already visited
        self._patrol_i = 0
        self._phase = "explore"  # explore -> collect -> deliver -> done
        self._goal = None        # (x, y) current goal
        self._goal_kind = None   # "explore" | "pick" | "depot"
        self._goal_sent = False
        self._goal_time = None   # wall-clock secs when the goal was sent

        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(PoseArray, "detected_crates", self._on_detections, 10)
        self.goal_pub = self.create_publisher(PoseStamped, "goal_pose", 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info("mission_node up: patrol -> discover -> collect.")

    # --- perception -------------------------------------------------
    def _on_odom(self, msg: Odometry) -> None:
        self._pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_detections(self, msg: PoseArray) -> None:
        for p in msg.poses:
            c = (p.position.x, p.position.y)
            if all(math.hypot(c[0] - k[0], c[1] - k[1]) > DEDUP_DIST for k in self._known):
                self._known.append(c)
                self.get_logger().info(
                    f"Vision: new crate at ({c[0]:+.2f}, {c[1]:+.2f}) "
                    f"[{len(self._known)} known].")

    # --- mission loop -----------------------------------------------
    def _tick(self) -> None:
        if self._pose is None or self._phase == "done":
            return
        if self._goal is None:
            self._choose_goal()
            return
        if not self._goal_sent:
            self._send_goal()
            return
        if math.hypot(self._goal[0] - self._pose[0], self._goal[1] - self._pose[1]) < ARRIVAL_TOLERANCE:
            self._on_arrival()
        elif self._goal_time is not None and \
                (self.get_clock().now().nanoseconds * 1e-9 - self._goal_time) > GOAL_TIMEOUT:
            self.get_logger().warn(
                f"[{self._phase}] goal {self._goal_kind} "
                f"({self._goal[0]:+.2f}, {self._goal[1]:+.2f}) timed out "
                f"after {GOAL_TIMEOUT:.0f}s -- abandoning, advancing.")
            self._abandon_goal()

    def _choose_goal(self) -> None:
        if self._phase == "explore":
            if self._patrol_i < len(PATROL):
                self._set_goal(PATROL[self._patrol_i], "explore")
                return
            self._phase = "collect"
        if self._phase == "collect":
            remaining = [c for c in self._known if c not in self._collected]
            if remaining:
                nearest = min(remaining, key=lambda c: math.hypot(
                    c[0] - self._pose[0], c[1] - self._pose[1]))
                self._set_goal(nearest, "pick")
                return
            self._phase = "deliver"
        if self._phase == "deliver":
            self._set_goal(DEPOT, "depot")

    def _on_arrival(self) -> None:
        kind = self._goal_kind
        if kind == "explore":
            self._patrol_i += 1
        elif kind == "pick":
            self._collected.append(self._goal)
            self.get_logger().info(
                f"At crate ({self._goal[0]:+.2f}, {self._goal[1]:+.2f}). "
                f"TODO: pick with the arm. [{len(self._collected)}/{len(self._known)}]")
        elif kind == "depot":
            self.get_logger().info(
                f"Delivered {len(self._collected)} crate(s) at the depot. Mission complete.")
            self._phase = "done"
        self._goal = None
        self._goal_sent = False
        self._goal_time = None

    def _abandon_goal(self) -> None:
        """Give up on an unreachable goal and move the mission forward."""
        kind = self._goal_kind
        if kind == "explore":
            self._patrol_i += 1
        elif kind == "pick":
            # Mark it "handled" so collect doesn't loop on it forever.
            self._collected.append(self._goal)
        # depot: just fall through and let deliver re-issue the goal.
        self._goal = None
        self._goal_sent = False
        self._goal_time = None

    def _set_goal(self, xy, kind) -> None:
        self._goal = xy
        self._goal_kind = kind
        self._goal_sent = False

    def _send_goal(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(self._goal[0])
        msg.pose.position.y = float(self._goal[1])
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self._goal_sent = True
        self._goal_time = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(
            f"[{self._phase}] goal {self._goal_kind} -> "
            f"({self._goal[0]:+.2f}, {self._goal[1]:+.2f}).")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionNode()
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
