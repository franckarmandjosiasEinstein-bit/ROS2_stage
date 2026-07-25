"""strawberry_detector -- colour-based ripe-strawberry detection for the camera.

Subscribes:
    /camera/image  (sensor_msgs/Image)   the robot's RGB camera (Gazebo)
Publishes:
    /camera/detections  (sensor_msgs/Image)  the same frame with green boxes
                                              drawn around detected ripe fruit
    /ripe_count         (std_msgs/Int32)     number of ripe clusters this frame

Perception is pure colour segmentation (lib/vision.red_mask) -- ideal for the
digital twin, where ripe berries are near-pure red. The exact same node
interface (subscribe an Image, publish detections) is what a YOLOv8 model would
later replace for real-world robustness, without touching the rest of the stack.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float32

from youbot_control.lib.vision import red_mask, blob_centroids


class StrawberryDetector(Node):
    def __init__(self) -> None:
        super().__init__("strawberry_detector")
        self.declare_parameter("image_topic", "/camera/image")
        self.declare_parameter("box_half", 10)     # px: half-size of the drawn box
        self.declare_parameter("min_pixels", 6)    # min blob size to count as fruit
        topic = self.get_parameter("image_topic").value
        self._box = int(self.get_parameter("box_half").value)
        self._min = int(self.get_parameter("min_pixels").value)

        self.create_subscription(Image, topic, self._on_image, 5)
        self.det_pub = self.create_publisher(Image, "camera/detections", 5)
        self.count_pub = self.create_publisher(Int32, "ripe_count", 5)
        # Horizontal offset of the fruit nearest the image centre, in [-1, 1]
        # (0 = a strawberry is directly beside the robot). 2.0 = none in view.
        self.offset_pub = self.create_publisher(Float32, "ripe_offset", 5)
        self.get_logger().info(
            f"strawberry_detector up: {topic} -> /camera/detections + /ripe_count + /ripe_offset")

    def _on_image(self, msg: Image):
        rgb = self._to_rgb(msg)
        if rgb is None:
            return
        mask = red_mask(rgb)
        centroids = blob_centroids(mask, min_pixels=self._min)

        annotated = rgb.copy()
        cx = msg.width / 2.0
        best_off = 2.0
        for u, v in centroids:
            self._draw_box(annotated, int(round(u)), int(round(v)))
            off = (u - cx) / cx
            if abs(off) < abs(best_off):
                best_off = off
        # Mark the centred target with a filled dot so it is obvious in the view.
        if centroids and abs(best_off) < 2.0:
            uc = int(round(best_off * cx + cx))
            self._draw_box(annotated, uc, int(round(
                min(centroids, key=lambda c: abs((c[0]-cx)/cx))[1])), colour=(255, 220, 0))

        self.det_pub.publish(self._to_msg(annotated, msg.header))
        self.count_pub.publish(Int32(data=len(centroids)))
        self.offset_pub.publish(Float32(data=float(best_off)))
        if centroids:
            self.get_logger().info(
                f"{len(centroids)} cluster(s), nearest offset {best_off:+.2f}.",
                throttle_duration_sec=2.0)

    # --- drawing --------------------------------------------------------
    def _draw_box(self, img, u, v, colour=(30, 230, 30)) -> None:
        h, w, _ = img.shape
        b = self._box
        u0, u1 = max(0, u - b), min(w - 1, u + b)
        v0, v1 = max(0, v - b), min(h - 1, v + b)
        img[v0, u0:u1 + 1] = colour       # top
        img[v1, u0:u1 + 1] = colour       # bottom
        img[v0:v1 + 1, u0] = colour       # left
        img[v0:v1 + 1, u1] = colour       # right

    # --- image <-> numpy ------------------------------------------------
    def _to_rgb(self, msg: Image):
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        enc = msg.encoding.lower()
        try:
            if enc in ("bgra8", "rgba8"):
                img = buf.reshape(msg.height, msg.width, 4)
                return img[:, :, [2, 1, 0]] if enc == "bgra8" else img[:, :, [0, 1, 2]]
            if enc in ("bgr8", "rgb8"):
                img = buf.reshape(msg.height, msg.width, 3)
                return img[:, :, [2, 1, 0]] if enc == "bgr8" else img
        except ValueError:
            return None
        self.get_logger().warn(f"Unsupported image encoding: {msg.encoding}", once=True)
        return None

    def _to_msg(self, rgb: np.ndarray, header) -> Image:
        out = Image()
        out.header = header
        out.height, out.width = rgb.shape[0], rgb.shape[1]
        out.encoding = "rgb8"
        out.is_bigendian = 0
        out.step = rgb.shape[1] * 3
        out.data = np.ascontiguousarray(rgb, dtype=np.uint8).tobytes()
        return out


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StrawberryDetector()
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
