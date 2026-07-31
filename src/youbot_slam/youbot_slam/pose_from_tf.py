"""pose_from_tf -- adapt slam_toolbox's TF output to the stack's pose topic.

slam_toolbox localises by publishing the map->odom TF correction (the ROS
convention), while our control stack consumes a full pose on a topic. This
adapter looks up the composed map->base_link transform at a fixed rate and
republishes it as nav_msgs/Odometry on /pose_slam -- the same topic the
homemade slam_node publishes, so the control stack cannot tell which SLAM
backend is running underneath it.

Subscribes:  TF (map -> odom -> base_link)
Publishes:   /pose_slam (nav_msgs/Odometry, frame map)
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener


class PoseFromTf(Node):
    def __init__(self) -> None:
        super().__init__("pose_from_tf")
        self.declare_parameter("rate", 20.0)
        self._buf = Buffer()
        self._listener = TransformListener(self._buf, self)
        self.pub = self.create_publisher(Odometry, "pose_slam", 20)
        self.create_timer(1.0 / float(self.get_parameter("rate").value), self._tick)
        self.get_logger().info("pose_from_tf up: TF map->base_link -> /pose_slam")

    def _tick(self) -> None:
        try:
            t = self._buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return                       # TF not available yet
        out = Odometry()
        out.header.stamp = t.header.stamp
        out.header.frame_id = "map"
        out.child_frame_id = "base_link"
        out.pose.pose.position.x = t.transform.translation.x
        out.pose.pose.position.y = t.transform.translation.y
        out.pose.pose.position.z = t.transform.translation.z
        out.pose.pose.orientation = t.transform.rotation
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseFromTf()
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
