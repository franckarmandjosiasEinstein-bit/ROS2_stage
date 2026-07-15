"""mission_node -- high-level orchestrator (simple state machine).

This is the ROS 2 counterpart of the Webots behaviour tree. To keep the
first scaffold approachable it is a plain state machine, not py_trees: it
walks a list of goal poses, publishing each to /goal_pose and advancing
when /odom shows the robot has arrived. Manipulation (pick / unload) is
stubbed with TODO markers -- wire it to an arm action server next.

When you are comfortable, swap this for `py_trees_ros` to get the same
Selector/Sequence tree you built in Webots. See docs/ARCHITECTURE.md.

Subscribes:  /odom (nav_msgs/Odometry)
Publishes:   /goal_pose (geometry_msgs/PoseStamped)
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# Demo mission: crate pick-up points then the depot. Replace with detections
# from a perception node (the Webots vision.py port) once that exists.
CRATES = [(0.5, 2.5), (-1.5, 0.0), (3.6, -1.57)]
DEPOT = (3.7, -4.5)
ARRIVAL_TOLERANCE = 0.25  # m


class MissionNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_node")
        # Build the step list: go to each crate (pick), then the depot (unload).
        self.steps = []
        for c in CRATES:
            self.steps.append(("pick", c))
        self.steps.append(("unload", DEPOT))
        self.index = 0
        self._pose = None
        self._goal_sent = False

        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.goal_pub = self.create_publisher(PoseStamped, "goal_pose", 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info(f"mission_node up: {len(self.steps)} steps queued.")

    def _on_odom(self, msg: Odometry) -> None:
        self._pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _tick(self) -> None:
        if self._pose is None or self.index >= len(self.steps):
            return
        action, (gx, gy) = self.steps[self.index]

        if not self._goal_sent:
            self._send_goal(gx, gy)
            self._goal_sent = True
            self.get_logger().info(f"Step {self.index + 1}/{len(self.steps)}: "
                                   f"{action} -> ({gx:+.2f}, {gy:+.2f}).")
            return

        if math.hypot(gx - self._pose[0], gy - self._pose[1]) < ARRIVAL_TOLERANCE:
            self.get_logger().info(f"Arrived. TODO: run '{action}' with the arm.")
            # TODO: call the manipulation action (pick / unload) and wait for it.
            self.index += 1
            self._goal_sent = False
            if self.index >= len(self.steps):
                self.get_logger().info("Mission complete.")

    def _send_goal(self, x, y) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
