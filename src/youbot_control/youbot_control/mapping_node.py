"""mapping_node -- builds an occupancy grid from the lidar.

Subscribes:
    /scan  (sensor_msgs/LaserScan)   the 360 deg lidar
    /odom  (nav_msgs/Odometry)       robot pose (from the Webots driver)
Publishes:
    /map      (nav_msgs/OccupancyGrid)  INFLATED grid, for the planner
    /map_raw  (nav_msgs/OccupancyGrid)  the evidence itself, for measuring

Why two. /map is grown by `inflation` (0.18 m) so A* can treat the robot as a
point -- every wall is 0.18 m thicker than it really is, and unknown cells are
indistinguishable from free ones. Scoring THAT against the world is how the
run of 2026-08-03 reported the greenhouse 20 cm too short with 13-17%
"clutter": it was measuring the safety margin, not the map. On /map_raw the
same greenhouse measures -4 cm / +10 cm with 0.0% clutter. /map_raw is the log-odds grid
thresholded and nothing else (-1 unknown, 0 free, 100 occupied), which is what
map_eval compares with the SDF.

The heavy lifting lives in `lib/occupancy_grid.py` (ported and validated in
Webots). This node is just the ROS 2 "glue": convert messages in, run the
algorithm, convert the grid out.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

from youbot_control.lib.occupancy_grid import OccupancyGrid


def yaw_from_quaternion(q) -> float:
    """Extract yaw (Z rotation) from a geometry_msgs Quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class MappingNode(Node):
    def __init__(self) -> None:
        super().__init__("mapping_node")
        self.declare_parameter("resolution", 0.10)
        self.declare_parameter("arena_size", 10.0)
        self.declare_parameter("inflation", 0.18)
        self.declare_parameter("publish_period", 0.5)

        res = self.get_parameter("resolution").value
        size = self.get_parameter("arena_size").value
        infl = self.get_parameter("inflation").value
        self.grid = OccupancyGrid(resolution=res, arena_size=size, inflation=infl)

        self._pose = (0.0, 0.0, 0.0)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "scan", self._on_scan, 10)
        self.map_pub = self.create_publisher(OccupancyGridMsg, "map", 1)
        self.raw_pub = self.create_publisher(OccupancyGridMsg, "map_raw", 1)
        self.create_timer(self.get_parameter("publish_period").value, self._publish_map)
        self.get_logger().info("mapping_node up: /scan + /odom -> /map "
                               "(inflated, for planning) + /map_raw (evidence)")

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def _on_scan(self, msg: LaserScan) -> None:
        x, y, yaw = self._pose
        self.grid.integrate_scan(list(msg.ranges), x, y, yaw,
                                 msg.angle_min, msg.angle_increment, msg.range_max)

    def _header(self, rows: int, cols: int) -> OccupancyGridMsg:
        msg = OccupancyGridMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = self.grid.resolution
        msg.info.width = cols
        msg.info.height = rows
        msg.info.origin.position.x = -self.grid.arena_size / 2.0
        msg.info.origin.position.y = -self.grid.arena_size / 2.0
        return msg

    def _publish_map(self) -> None:
        self.grid.update_binary()
        rows, cols = self.grid.shape
        msg = self._header(rows, cols)
        # ROS OccupancyGrid: row 0 is the bottom (y = origin). Our grid has
        # row 0 at the top, so flip vertically. 0 = free, 100 = occupied.
        msg.data = (self.grid.grid[::-1, :] * 100).astype("int8").flatten().tolist()
        self.map_pub.publish(msg)

        # The evidence, ungrown and with unknown kept distinct from free --
        # "we never looked there" and "we looked and it was empty" are not the
        # same claim, and coverage is only meaningful if they differ.
        lo = self.grid.log_odds[::-1, :]
        raw = np.full(lo.shape, -1, dtype="int8")
        raw[lo < -0.2] = 0
        raw[lo > 0.2] = 100
        m = self._header(rows, cols)
        m.data = raw.flatten().tolist()
        self.raw_pub.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MappingNode()
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
