"""mapping_node -- builds an occupancy grid from the lidar.

Subscribes:
    /scan  (sensor_msgs/LaserScan)   the 360 deg lidar
    /odom  (nav_msgs/Odometry)       robot pose (from the Webots driver)
Publishes:
    /map   (nav_msgs/OccupancyGrid)  inflated occupancy grid for planning

The heavy lifting lives in `lib/occupancy_grid.py` (ported and validated in
Webots). This node is just the ROS 2 "glue": convert messages in, run the
algorithm, convert the grid out.
"""

from __future__ import annotations

import math

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
        self.declare_parameter("inflation", 0.32)
        self.declare_parameter("publish_period", 0.5)

        res = self.get_parameter("resolution").value
        size = self.get_parameter("arena_size").value
        infl = self.get_parameter("inflation").value
        self.grid = OccupancyGrid(resolution=res, arena_size=size, inflation=infl)

        self._pose = (0.0, 0.0, 0.0)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "scan", self._on_scan, 10)
        self.map_pub = self.create_publisher(OccupancyGridMsg, "map", 1)
        self.create_timer(self.get_parameter("publish_period").value, self._publish_map)
        self.get_logger().info("mapping_node up: /scan + /odom -> /map")

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def _on_scan(self, msg: LaserScan) -> None:
        x, y, yaw = self._pose
        self.grid.integrate_scan(list(msg.ranges), x, y, yaw,
                                 msg.angle_min, msg.angle_increment, msg.range_max)

    def _publish_map(self) -> None:
        self.grid.update_binary()
        msg = OccupancyGridMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = self.grid.resolution
        rows, cols = self.grid.shape
        msg.info.width = cols
        msg.info.height = rows
        msg.info.origin.position.x = -self.grid.arena_size / 2.0
        msg.info.origin.position.y = -self.grid.arena_size / 2.0
        # ROS OccupancyGrid: row 0 is the bottom (y = origin). Our grid has
        # row 0 at the top, so flip vertically. 0 = free, 100 = occupied.
        data = (self.grid.grid[::-1, :] * 100).astype("int8").flatten()
        msg.data = data.tolist()
        self.map_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MappingNode()
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
