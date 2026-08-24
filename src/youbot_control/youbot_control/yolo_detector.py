"""yolo_detector -- YOLOv8-based strawberry detection for real-world robustness.

Drop-in replacement for the colour-threshold strawberry_detector: same
subscriptions, same published topics, same map-projection logic. The only
difference is how a fruit is found in the image: a learned model instead of
a channel-difference threshold.

The colour detector works well in Gazebo (69% recall) but fails under real
daylight because its red-channel test is invariant to illumination SCALE but
not to illumination COLOUR. A learned model trained on varied lighting
conditions (see ml/README.md) fixes the class of problem.

Requires: pip install ultralytics

Subscribes:
    /camera/image       (sensor_msgs/Image)   the robot's RGB camera
Publishes:
    /camera/detections  (sensor_msgs/Image)   annotated frame with boxes
    /ripe_count         (std_msgs/Int32)      ripe strawberry count this frame
    /ripe_offset        (std_msgs/Float32)    horizontal offset of nearest ripe
    /ripe_offsets        (std_msgs/Float32MultiArray)  all ripe offsets
    /berry_detections   (geometry_msgs/PoseArray)  map positions of detections
"""

from __future__ import annotations

import math
import os

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Float32MultiArray, Float64, Int32


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class YoloDetector(Node):
    def __init__(self) -> None:
        super().__init__("strawberry_detector")
        self.declare_parameter("image_topic", "/camera/image")
        self.declare_parameter("weights", "")
        self.declare_parameter("confidence", 0.35)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "")

        self.declare_parameter("cam_x", 0.18)
        self.declare_parameter("cam_y", 0.0)
        self.declare_parameter("cam_arm", 0.14)
        self.declare_parameter("cam_z", 0.78)
        self.declare_parameter("cam_pitch", 0.28)
        self.declare_parameter("cam_yaw", math.pi / 2.0)
        self.declare_parameter("require_pan_settled", True)
        self.declare_parameter("cam_fov", 1.2)
        self.declare_parameter("berry_z", 0.97)
        self.declare_parameter("max_range", 2.5)
        self.declare_parameter("max_pixel_shift", 0.15)

        self._model = None
        weights = str(self.get_parameter("weights").value)
        if weights and os.path.exists(weights):
            try:
                from ultralytics import YOLO
                conf = float(self.get_parameter("confidence").value)
                self._model = YOLO(weights)
                self._conf = conf
                self._imgsz = int(self.get_parameter("imgsz").value)
                dev = str(self.get_parameter("device").value)
                self._device = dev if dev else None
                self.get_logger().info(
                    f"YOLOv8 model loaded from {weights} (conf={conf:.2f})")
            except ImportError:
                self.get_logger().error(
                    "ultralytics not installed — falling back to colour detection. "
                    "Install with: pip install ultralytics")
            except Exception as e:
                self.get_logger().error(
                    f"Failed to load YOLO model from {weights}: {e}")
        else:
            self.get_logger().warn(
                "No YOLO weights provided or file not found — "
                "falling back to colour-threshold detection. "
                "Set the 'weights' parameter to a .pt file path.")

        if self._model is None:
            from youbot_control.lib.vision import red_mask, close_mask, blob_centroids
            self._red_mask = red_mask
            self._close_mask = close_mask
            self._blob_centroids = blob_centroids

        topic = self.get_parameter("image_topic").value
        self.create_subscription(Image, topic, self._on_image, 5)
        self.det_pub = self.create_publisher(Image, "camera/detections", 5)
        self.mask_pub = self.create_publisher(Image, "camera/ripe_mask", 5)
        self.count_pub = self.create_publisher(Int32, "ripe_count", 5)
        self.offset_pub = self.create_publisher(Float32, "ripe_offset", 5)
        self.offsets_pub = self.create_publisher(Float32MultiArray, "ripe_offsets", 5)
        self.world_pub = self.create_publisher(PoseArray, "berry_detections", 5)

        self._pose = None
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self._pan = None
        self._pan_settled = False
        self.create_subscription(Float64, "camera_pan_state", self._on_pan, 5)
        self.create_subscription(Bool, "camera_pan_settled",
                                 self._on_pan_settled, 5)

        mode = "YOLOv8" if self._model else "colour fallback"
        self.get_logger().info(
            f"yolo_detector up ({mode}): {topic} -> /camera/detections "
            "+ /ripe_count + /ripe_offset + /berry_detections")

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._pose = (p.position.x, p.position.y,
                      yaw_from_quaternion(p.orientation))

    def _on_pan(self, msg) -> None:
        self._pan = float(msg.data)

    def _on_pan_settled(self, msg) -> None:
        self._pan_settled = bool(msg.data)

    def _pan_angle(self):
        if self._pan is None:
            if bool(self.get_parameter("require_pan_settled").value):
                return None
            return float(self.get_parameter("cam_yaw").value)
        if (bool(self.get_parameter("require_pan_settled").value)
                and not self._pan_settled):
            return None
        return self._pan

    def _ray_hit(self, u, v, width, height):
        if self._pose is None:
            return None
        rx, ry, ryaw = self._pose
        g = self.get_parameter
        pitch = float(g("cam_pitch").value)
        pan = self._pan_angle()
        if pan is None:
            return None
        a = ryaw + pan
        fov = float(g("cam_fov").value)
        fx = (width / 2.0) / math.tan(fov / 2.0)
        ca, sa, cp, sp = math.cos(a), math.sin(a), math.cos(pitch), math.sin(pitch)
        f = (ca * cp, sa * cp, sp)
        right = (sa, -ca, 0.0)
        down = (f[1] * right[2] - f[2] * right[1],
                f[2] * right[0] - f[0] * right[2],
                f[0] * right[1] - f[1] * right[0])
        xc = (u - width / 2.0) / fx
        yc = (v - height / 2.0) / fx
        d = tuple(f[i] + xc * right[i] + yc * down[i] for i in range(3))
        arm = float(g("cam_arm").value)
        bx = float(g("cam_x").value) + arm * math.cos(pan)
        by = float(g("cam_y").value) + arm * math.sin(pan)
        cam = (rx + bx * math.cos(ryaw) - by * math.sin(ryaw),
               ry + bx * math.sin(ryaw) + by * math.cos(ryaw),
               float(g("cam_z").value))
        if abs(d[2]) < 1e-3:
            return None
        t = (float(g("berry_z").value) - cam[2]) / d[2]
        if t <= 0.0:
            return None
        horiz = math.hypot(t * d[0], t * d[1])
        if horiz > float(g("max_range").value):
            return None
        return cam[0] + t * d[0], cam[1] + t * d[1]

    def _project(self, u, v, width, height):
        p = self._ray_hit(u, v, width, height)
        if p is None:
            return None
        q = self._ray_hit(u, v + 1.0, width, height)
        if q is None:
            return None
        if math.hypot(q[0] - p[0], q[1] - p[1]) > float(
                self.get_parameter("max_pixel_shift").value):
            return None
        return p

    def _on_image(self, msg: Image):
        rgb = self._to_rgb(msg)
        if rgb is None:
            return

        if self._model is not None:
            self._detect_yolo(rgb, msg)
        else:
            self._detect_colour(rgb, msg)

    def _detect_yolo(self, rgb: np.ndarray, msg: Image):
        results = self._model.predict(
            rgb, imgsz=self._imgsz, conf=self._conf,
            device=self._device, verbose=False)
        if not results:
            self._publish_empty(msg)
            return

        res = results[0]
        annotated = rgb.copy()
        cx = msg.width / 2.0
        best_off = 2.0
        offsets = []
        world = PoseArray()
        world.header.stamp = msg.header.stamp
        world.header.frame_id = "map"
        ripe_count = 0

        for box in res.boxes:
            cls = int(box.cls.item())
            conf = float(box.conf.item())
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            if cls == 0:
                colour = (30, 230, 30)
                label = f"ripe {conf:.0%}"
                ripe_count += 1
                u = (x1 + x2) / 2.0
                v = (y1 + y2) / 2.0
                off = (u - cx) / cx
                offsets.append(float(off))
                if abs(off) < abs(best_off):
                    best_off = off
                xy = self._project(u, v, msg.width, msg.height)
                if xy is not None:
                    p = Pose()
                    p.position.x, p.position.y = float(xy[0]), float(xy[1])
                    p.position.z = float(self.get_parameter("berry_z").value)
                    p.orientation.w = 1.0
                    world.poses.append(p)
            elif cls == 1:
                colour = (100, 200, 100)
                label = f"unripe {conf:.0%}"
            elif cls == 2:
                colour = (50, 180, 50)
                label = f"plant {conf:.0%}"
            elif cls == 3:
                colour = (200, 200, 50)
                label = f"glass {conf:.0%}"
            else:
                colour = (180, 180, 180)
                label = f"c{cls} {conf:.0%}"

            self._draw_rect(annotated, x1, y1, x2, y2, colour)
            self._draw_label(annotated, label, x1, max(0, y1 - 2), colour)

        self.det_pub.publish(self._to_msg(annotated, msg.header))
        self.count_pub.publish(Int32(data=ripe_count))
        self.offset_pub.publish(Float32(data=float(best_off)))
        self.offsets_pub.publish(Float32MultiArray(data=sorted(offsets, key=abs)))
        self.world_pub.publish(world)

        if ripe_count:
            self.get_logger().info(
                f"YOLO: {ripe_count} ripe, nearest offset {best_off:+.2f}, "
                f"{len(world.poses)} located on map.",
                throttle_duration_sec=2.0)
        else:
            n_total = len(res.boxes)
            self.get_logger().info(
                f"YOLO: 0 ripe ({n_total} other detections).",
                throttle_duration_sec=5.0)

    def _detect_colour(self, rgb: np.ndarray, msg: Image):
        mask = self._red_mask(rgb)
        mask = self._close_mask(mask, 2)
        centroids = self._blob_centroids(mask, min_pixels=6, min_fill=0.45,
                                         max_aspect=3.0)

        annotated = rgb.copy()
        cx = msg.width / 2.0
        best_off = 2.0
        offsets = []
        world = PoseArray()
        world.header.stamp = msg.header.stamp
        world.header.frame_id = "map"
        for u, v in centroids:
            self._draw_box(annotated, int(round(u)), int(round(v)), 10)
            off = (u - cx) / cx
            offsets.append(float(off))
            if abs(off) < abs(best_off):
                best_off = off
            xy = self._project(u, v, msg.width, msg.height)
            if xy is not None:
                p = Pose()
                p.position.x, p.position.y = float(xy[0]), float(xy[1])
                p.position.z = float(self.get_parameter("berry_z").value)
                p.orientation.w = 1.0
                world.poses.append(p)

        self.det_pub.publish(self._to_msg(annotated, msg.header))
        self.mask_pub.publish(self._to_msg(
            np.repeat((mask[:, :, None] * np.uint8(255)), 3, axis=2), msg.header))
        self.count_pub.publish(Int32(data=len(centroids)))
        self.offset_pub.publish(Float32(data=float(best_off)))
        self.offsets_pub.publish(Float32MultiArray(data=sorted(offsets, key=abs)))
        self.world_pub.publish(world)

        if centroids:
            self.get_logger().info(
                f"colour fallback: {len(centroids)} cluster(s), "
                f"nearest offset {best_off:+.2f}.",
                throttle_duration_sec=2.0)

    def _publish_empty(self, msg: Image):
        self.count_pub.publish(Int32(data=0))
        self.offset_pub.publish(Float32(data=2.0))
        self.offsets_pub.publish(Float32MultiArray(data=[]))
        world = PoseArray()
        world.header.stamp = msg.header.stamp
        world.header.frame_id = "map"
        self.world_pub.publish(world)

    def _draw_rect(self, img, x1, y1, x2, y2, colour=(30, 230, 30)):
        h, w, _ = img.shape
        x1, x2 = max(0, x1), min(w - 1, x2)
        y1, y2 = max(0, y1), min(h - 1, y2)
        img[y1, x1:x2 + 1] = colour
        img[y2, x1:x2 + 1] = colour
        img[y1:y2 + 1, x1] = colour
        img[y1:y2 + 1, x2] = colour

    def _draw_label(self, img, text, x, y, colour=(30, 230, 30)):
        h, w, _ = img.shape
        for i, ch in enumerate(text[:20]):
            px = x + i * 6
            if 0 <= px < w - 5 and 0 <= y < h - 8:
                img[y:y + 2, px:px + 5] = colour

    def _draw_box(self, img, u, v, half, colour=(30, 230, 30)):
        h, w, _ = img.shape
        u0, u1 = max(0, u - half), min(w - 1, u + half)
        v0, v1 = max(0, v - half), min(h - 1, v + half)
        img[v0, u0:u1 + 1] = colour
        img[v1, u0:u1 + 1] = colour
        img[v0:v1 + 1, u0] = colour
        img[v0:v1 + 1, u1] = colour

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
    node = YoloDetector()
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
