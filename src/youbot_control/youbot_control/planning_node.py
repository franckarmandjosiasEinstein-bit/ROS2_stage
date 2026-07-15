"""planning_node -- A* path planning from the current pose to a goal.

Subscribes:
    /map        (nav_msgs/OccupancyGrid)  from mapping_node
    /odom       (nav_msgs/Odometry)       current pose (path start)
    /goal_pose  (geometry_msgs/PoseStamped)  where to go (e.g. from RViz "2D Goal Pose")
Publishes:
    /plan       (nav_msgs/Path)           the A* waypoint list

Algorithm ported from the validated Webots `planning.py` (lib/astar.py).
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path

from youbot_control.lib import astar


class PlanningNode(Node):
    def __init__(self) -> None:
        super().__init__("planning_node")
        self.declare_parameter("resolution", 0.10)
        self.declare_parameter("arena_size", 10.0)
        self.resolution = self.get_parameter("resolution").value
        self.arena_size = self.get_parameter("arena_size").value

        self._grid = None
        self._start = (0.0, 0.0)
        self.create_subscription(OccupancyGrid, "map", self._on_map, 1)
        self.create_subscription(PoseStamped, "goal_pose", self._on_goal, 10)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.plan_pub = self.create_publisher(Path, "plan", 1)
        self.get_logger().info("planning_node up: /map + /goal_pose -> /plan")

    def _on_odom(self, msg) -> None:
        self._start = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_map(self, msg: OccupancyGrid) -> None:
        # Rebuild the numpy grid (flip back: ROS row 0 is bottom, ours is top).
        arr = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self._grid = (arr[::-1, :] > 50).astype(np.uint8)
        self.resolution = msg.info.resolution

    def _on_goal(self, msg: PoseStamped) -> None:
        if self._grid is None:
            self.get_logger().warn("No /map yet; cannot plan.")
            return
        goal = (msg.pose.position.x, msg.pose.position.y)
        waypoints = astar.plan(self._grid, self.resolution, self.arena_size,
                               self._start, goal)
        if not waypoints:
            self.get_logger().warn(f"No path from {self._start} to {goal}.")
            return
        self.plan_pub.publish(self._to_path(waypoints))
        self.get_logger().info(f"Plan: {len(waypoints)} waypoints to {goal}.")

    def _to_path(self, waypoints) -> Path:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        for x, y in waypoints:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        return path


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
