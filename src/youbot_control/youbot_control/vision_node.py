"""vision_node -- detect red crates from the camera (Webots real perception).

Subscribes:
    /camera/image_raw/image_color  (sensor_msgs/Image)  the RGB camera
    /odom                          (nav_msgs/Odometry)  robot pose
Publishes:
    /detected_crates  (geometry_msgs/PoseArray)     crates seen this frame
    /crate_markers    (visualization_msgs/MarkerArray)  accumulated, for RViz

Detection + ground projection come from lib/vision.py (ported and validated
in Webots). This node is the ROS glue: decode the image, run the pipeline
with the robot pose, publish. Use it with the Webots driver; headless, the
sim_node already emits /detected_crates via its FOV cone.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray

from youbot_control.lib.vision import red_mask, blob_centroids, GroundProjector


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class VisionNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_node")
        self.declare_parameter("image_topic", "/camera/image_raw/image_color")
        self.declare_parameter("fov", 1.2)
        self.declare_parameter("dedup_dist", 0.6)
        topic = self.get_parameter("image_topic").value
        self._fov = float(self.get_parameter("fov").value)
        self._dedup = float(self.get_parameter("dedup_dist").value)

        self._pose = (0.0, 0.0, 0.0)
        self._proj = None
        self._known = []

        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(Image, topic, self._on_image, 5)
        self.detect_pub = self.create_publisher(PoseArray, "detected_crates", 5)
        self.marker_pub = self.create_publisher(MarkerArray, "crate_markers", 1)
        self.get_logger().info(f"vision_node up: {topic} + /odom -> /detected_crates")

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def _on_image(self, msg: Image) -> None:
        rgb = self._to_rgb(msg)
        if rgb is None:
            return
        if self._proj is None:
            self._proj = GroundProjector(msg.width, msg.height, self._fov)

        mask = red_mask(rgb)
        rx, ry, yaw = self._pose
        found = []
        for u, v in blob_centroids(mask):
            world = self._proj.pixel_to_ground(u, v, rx, ry, yaw)
            if world is not None:
                found.append(world)

        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = "map"
        for wx, wy in found:
            p = Pose()
            p.position.x, p.position.y, p.orientation.w = wx, wy, 1.0
            pa.poses.append(p)
            if all(math.hypot(wx - k[0], wy - k[1]) > self._dedup for k in self._known):
                self._known.append((wx, wy))
                self.get_logger().info(f"Vision: crate at ({wx:+.2f}, {wy:+.2f}).")
        self.detect_pub.publish(pa)
        self._publish_markers()

    def _to_rgb(self, msg: Image):
        """Decode common encodings to an (H, W, 3) R,G,B uint8 array."""
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        enc = msg.encoding.lower()
        try:
            if enc in ("bgra8", "rgba8"):
                img = buf.reshape(msg.height, msg.width, 4)
                if enc == "bgra8":
                    return img[:, :, [2, 1, 0]]
                return img[:, :, [0, 1, 2]]
            if enc in ("bgr8", "rgb8"):
                img = buf.reshape(msg.height, msg.width, 3)
                if enc == "bgr8":
                    return img[:, :, [2, 1, 0]]
                return img
        except ValueError:
            return None
        self.get_logger().warn(f"Unsupported image encoding: {msg.encoding}", once=True)
        return None

    def _publish_markers(self) -> None:
        arr = MarkerArray()
        for i, (cx, cy) in enumerate(self._known):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "crates"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.1, 0.1, 1.0
            arr.markers.append(m)
        self.marker_pub.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionNode()
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
