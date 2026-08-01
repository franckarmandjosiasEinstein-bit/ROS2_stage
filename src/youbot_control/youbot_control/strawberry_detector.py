"""strawberry_detector -- colour-based ripe-strawberry detection for the camera.

Subscribes:
    /camera/image  (sensor_msgs/Image)   the robot's RGB camera (Gazebo)
Publishes:
    /camera/detections  (sensor_msgs/Image)  the same frame with green boxes
                                              drawn around detected ripe fruit
    /ripe_count         (std_msgs/Int32)     number of ripe clusters this frame

Perception is pure colour segmentation (lib/vision.red_mask). The exact same
node interface (subscribe an Image, publish detections) is what a YOLOv8 model
would later replace for real-world robustness, without touching the rest of
the stack.

Every threshold is a ROS parameter and the node reports what it is seeing even
when it sees nothing:

    ros2 param set /strawberry_detector min_diff 35
    ros2 topic echo /ripe_count

When a frame yields no fruit the log prints the reddest pixel actually present
(`best r-g`). That one number says whether the thresholds are wrong or the
camera simply is not pointing at a plant -- the field logs said "15 berries in
view, vision reports 0" for a whole run and there was no way to tell which.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float32, Float32MultiArray

from youbot_control.lib.vision import red_mask, close_mask, blob_centroids


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class StrawberryDetector(Node):
    def __init__(self) -> None:
        super().__init__("strawberry_detector")
        self.declare_parameter("image_topic", "/camera/image")
        self.declare_parameter("box_half", 10)     # px: half-size of the drawn box
        self.declare_parameter("min_pixels", 6)    # min blob size to count as fruit
        # Ripeness thresholds (see lib/vision.red_mask for why they are
        # differences and not ratios).
        self.declare_parameter("value_min", 55)    # ignore near-black pixels
        self.declare_parameter("min_diff", 45)     # how far red must lead g and b
        self.declare_parameter("max_sym", 0.20)    # |g-b| <= this * r  (rejects orange)
        self.declare_parameter("max_ratio", 0.62)  # g <= this * r      (rejects pink)
        # Shape clean-up: close gaps a leaf cuts across a berry, then demand
        # the blob be round-ish. One fruit should count once, and a red sliver
        # along a gutter edge should count as none.
        self.declare_parameter("close_iterations", 2)  # bridges gaps up to 4 px
        self.declare_parameter("min_fill", 0.45)   # blob pixels / bounding box
        self.declare_parameter("max_aspect", 3.0)  # longer box side / shorter
        # --- where the fruit IS, not just where it is on the screen ---------
        # A pixel alone gives a ray, not a point. The missing constraint here
        # is height: strawberries hang off gutters whose soil line is fixed by
        # the greenhouse structure, so they all sit in a narrow band around
        # berry_z. Intersecting the pixel ray with that plane turns every
        # detection into an (x, y) in the map -- which is what makes the
        # perception debuggable: those estimates can be laid over the true
        # berry catalogue and the error read off directly, instead of judging
        # a detector by a count that says nothing about WHERE it looked.
        # Mount and optics must match the URDF sensor block.
        self.declare_parameter("cam_x", 0.18)      # m, base frame
        self.declare_parameter("cam_y", 0.14)
        self.declare_parameter("cam_z", 0.78)
        self.declare_parameter("cam_pitch", 0.28)  # rad, positive = tilted up
        self.declare_parameter("cam_yaw", math.pi / 2.0)   # rad from the heading
        self.declare_parameter("cam_fov", 1.2)     # rad, horizontal
        self.declare_parameter("berry_z", 0.97)    # m: the fruit plane
        self.declare_parameter("max_range", 2.5)   # m: refuse distant guesses
        topic = self.get_parameter("image_topic").value
        self._box = int(self.get_parameter("box_half").value)
        self._min = int(self.get_parameter("min_pixels").value)

        self.create_subscription(Image, topic, self._on_image, 5)
        self.det_pub = self.create_publisher(Image, "camera/detections", 5)
        # The segmentation itself, as an image. Tuning a colour threshold from
        # a count alone is guesswork; seeing which pixels survived is not.
        self.mask_pub = self.create_publisher(Image, "camera/ripe_mask", 5)
        self.count_pub = self.create_publisher(Int32, "ripe_count", 5)
        # Horizontal offset of the fruit nearest the image centre, in [-1, 1]
        # (0 = a strawberry is directly beside the robot). 2.0 = none in view.
        self.offset_pub = self.create_publisher(Float32, "ripe_offset", 5)
        # ALL cluster offsets this frame (same [-1, 1] convention) -- lets the
        # mission target the NEXT berry at a station, not just the nearest.
        self.offsets_pub = self.create_publisher(Float32MultiArray, "ripe_offsets", 5)
        # Estimated MAP positions of the fruit seen this frame. truth_monitor
        # accumulates these and prints them against the real catalogue.
        self.world_pub = self.create_publisher(PoseArray, "berry_detections", 5)
        self._pose = None            # robot (x, y, yaw); no pose -> no map fix
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.get_logger().info(
            f"strawberry_detector up: {topic} -> /camera/detections + /ripe_count "
            "+ /ripe_offset + /berry_detections (map positions)")

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._pose = (p.position.x, p.position.y,
                      yaw_from_quaternion(p.orientation))

    def _project(self, u, v, width, height):
        """Pixel -> map (x, y), by intersecting the ray with the fruit plane.

        Returns None when the ray never reaches the plane (pointing away from
        it, or so shallow that the estimate would be meaningless) or lands
        beyond max_range, where a 1 px error is already tens of centimetres."""
        if self._pose is None:
            return None
        rx, ry, ryaw = self._pose
        g = self.get_parameter
        pitch = float(g("cam_pitch").value)
        a = ryaw + float(g("cam_yaw").value)
        fov = float(g("cam_fov").value)
        fx = (width / 2.0) / math.tan(fov / 2.0)      # square pixels

        # Camera basis in the map frame: optical axis, image-right, image-down.
        ca, sa, cp, sp = math.cos(a), math.sin(a), math.cos(pitch), math.sin(pitch)
        f = (ca * cp, sa * cp, sp)
        right = (sa, -ca, 0.0)                         # f x up, normalised
        down = (f[1] * right[2] - f[2] * right[1],     # f x right
                f[2] * right[0] - f[0] * right[2],
                f[0] * right[1] - f[1] * right[0])

        xc = (u - width / 2.0) / fx
        yc = (v - height / 2.0) / fx
        d = tuple(f[i] + xc * right[i] + yc * down[i] for i in range(3))

        cam = (rx + float(g("cam_x").value) * math.cos(ryaw)
               - float(g("cam_y").value) * math.sin(ryaw),
               ry + float(g("cam_x").value) * math.sin(ryaw)
               + float(g("cam_y").value) * math.cos(ryaw),
               float(g("cam_z").value))
        if abs(d[2]) < 1e-3:
            return None                                # ray parallel to the plane
        t = (float(g("berry_z").value) - cam[2]) / d[2]
        if t <= 0.0:
            return None
        horiz = math.hypot(t * d[0], t * d[1])
        if horiz > float(g("max_range").value):
            return None
        return cam[0] + t * d[0], cam[1] + t * d[1]

    def _on_image(self, msg: Image):
        rgb = self._to_rgb(msg)
        if rgb is None:
            return
        mask = red_mask(rgb,
                        value_min=int(self.get_parameter("value_min").value),
                        min_diff=int(self.get_parameter("min_diff").value),
                        max_sym=float(self.get_parameter("max_sym").value),
                        max_ratio=float(self.get_parameter("max_ratio").value))
        mask = close_mask(mask, int(self.get_parameter("close_iterations").value))
        centroids = blob_centroids(
            mask, min_pixels=self._min,
            min_fill=float(self.get_parameter("min_fill").value),
            max_aspect=float(self.get_parameter("max_aspect").value))

        annotated = rgb.copy()
        cx = msg.width / 2.0
        best_off = 2.0
        offsets = []
        world = PoseArray()
        world.header.stamp = msg.header.stamp
        world.header.frame_id = "map"
        for u, v in centroids:
            self._draw_box(annotated, int(round(u)), int(round(v)))
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
        # Mark the centred target with a filled dot so it is obvious in the view.
        if centroids and abs(best_off) < 2.0:
            uc = int(round(best_off * cx + cx))
            self._draw_box(annotated, uc, int(round(
                min(centroids, key=lambda c: abs((c[0]-cx)/cx))[1])), colour=(255, 220, 0))

        self.det_pub.publish(self._to_msg(annotated, msg.header))
        self.mask_pub.publish(self._to_msg(
            np.repeat((mask[:, :, None] * np.uint8(255)), 3, axis=2), msg.header))
        self.count_pub.publish(Int32(data=len(centroids)))
        self.offset_pub.publish(Float32(data=float(best_off)))
        self.offsets_pub.publish(Float32MultiArray(data=sorted(offsets, key=abs)))
        self.world_pub.publish(world)
        if centroids:
            where = (f", {len(world.poses)} located on the map"
                     if world.poses else
                     ", none located (no pose yet)" if self._pose is None else
                     ", none located (ray misses the fruit plane)")
            self.get_logger().info(
                f"{len(centroids)} cluster(s), nearest offset {best_off:+.2f}"
                f"{where}.",
                throttle_duration_sec=2.0)
        else:
            # Nothing found: say what the frame actually contained, so a wrong
            # threshold is distinguishable from a camera aimed at a wall.
            r = rgb[:, :, 0].astype(np.int16)
            lead = int(np.max(r - np.maximum(rgb[:, :, 1], rgb[:, :, 2]).astype(np.int16)))
            self.get_logger().info(
                f"no fruit: {int(mask.sum())} px pass the mask, reddest pixel in "
                f"frame leads g/b by {lead} (threshold {self.get_parameter('min_diff').value}).",
                throttle_duration_sec=5.0)

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
    except (KeyboardInterrupt, RuntimeError):
        pass      # RuntimeError: message mid-shutdown (rclpy)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
