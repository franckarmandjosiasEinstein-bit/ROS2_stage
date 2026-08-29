"""scan_relay -- QoS bridge for /scan so slam_toolbox receives every scan.

ros_gz_bridge publishes sensor topics with SensorDataQoS (BEST_EFFORT
reliability).  slam_toolbox's internal message_filters::Subscriber uses the
system default (RELIABLE).  A RELIABLE subscriber silently refuses data from
a BEST_EFFORT publisher, so slam_toolbox never sees a single scan.

This node subscribes with BEST_EFFORT (compatible with any publisher) and
republishes with RELIABLE (compatible with slam_toolbox), closing the gap.

Subscribes:  /scan            (sensor_msgs/LaserScan, any QoS)
Publishes:   /scan_reliable   (sensor_msgs/LaserScan, RELIABLE)
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from sensor_msgs.msg import LaserScan


class ScanRelay(Node):
    def __init__(self) -> None:
        super().__init__("scan_relay")

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._pub = self.create_publisher(LaserScan, "scan_reliable", reliable_qos)
        self.create_subscription(LaserScan, "scan", self._on_scan, sensor_qos)
        self._count = 0
        self.create_timer(5.0, self._diag)
        self.get_logger().info(
            "scan_relay up: /scan (BEST_EFFORT) -> /scan_reliable (RELIABLE)")

    def _on_scan(self, msg: LaserScan) -> None:
        self._count += 1
        self._pub.publish(msg)

    def _diag(self) -> None:
        self.get_logger().info(f"scan_relay diag: relayed {self._count} scans")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanRelay()
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
