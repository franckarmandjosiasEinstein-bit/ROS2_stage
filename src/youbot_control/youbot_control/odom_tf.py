"""odom_tf -- broadcast the map->base_link transform from /odom.

In Gazebo, localisation comes from the ground-truth /odom (bridged from the
OdometryPublisher). We rebroadcast it as TF here rather than bridging the gz
Pose_V TF directly: that bridge can deliver the transform on WALL time while the
rest of the graph runs on sim time, which makes RViz thrash ("Detected jump back
in time") and lose the lidar->map lookup. Restamping from /odom (whose header is
on sim time) keeps TF, /scan and /clock on one clock.

Subscribes:  /odom (nav_msgs/Odometry)
Broadcasts:  TF  map -> base_link
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTf(Node):
    def __init__(self) -> None:
        super().__init__("odom_tf")
        self._br = TransformBroadcaster(self)
        self.create_subscription(Odometry, "odom", self._on_odom, 20)
        self.get_logger().info("odom_tf up: /odom -> TF map->base_link")

    def _on_odom(self, msg: Odometry) -> None:
        t = TransformStamped()
        t.header = msg.header                       # sim-time stamp, frame_id 'map'
        t.child_frame_id = msg.child_frame_id or "base_link"
        p = msg.pose.pose
        t.transform.translation.x = p.position.x
        t.transform.translation.y = p.position.y
        t.transform.translation.z = p.position.z
        t.transform.rotation = p.orientation
        self._br.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomTf()
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
