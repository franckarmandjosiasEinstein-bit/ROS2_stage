"""dataset_capture -- build an annotated training set from the digital twin.

THE PROBLEM THIS SOLVES

The colour-threshold detector measures 69% recall with two of every three map
estimates spurious, and it cannot be repaired by tuning: a ratio test is
invariant to illumination SCALE, which is why it survives shade, but not to
illumination COLOUR, and late-afternoon light through glass lowers G/R for the
entire scene. Replacing it needs a learned model, and a learned model needs
annotated images. The greenhouse is not reachable for photography.

But the twin knows where all 127 berries are. So it can write its own labels.
That is the one thing a digital twin gives that a photographer does not: not
pictures -- pictures are easy -- but pictures with PERFECT ground truth,
thousands of them, overnight, for free.

WHERE THE LABELS COME FROM

Not from the colour detector. Using the detector to label the data that trains
its replacement would teach the replacement to reproduce its errors exactly,
including the two-thirds false positives. That is the circularity to avoid, and
it is easy to fall into because it is so convenient.

Labels come from the berry catalogue written by make_world.py, projected
through youbot_slam.lib.berry_view -- the SAME visibility model truth_monitor
uses to score. Scorer and labeller share one implementation, so a model cannot
be trained to find one set of berries and marked on another.

WHAT IS KEPT, AND WHAT IS THROWN AWAY

Two filters, both about the honesty of the set rather than its size.

  * Frames are kept only when the robot has MOVED enough since the last one.
    A camera at 15 Hz on a robot at 0.2 m/s produces twenty near-identical
    pictures of the same plant. Train on nineteen and test on the twentieth
    and the model scores 99% and fails in the field.
  * Frames are dropped while the pan head is moving. The head angle enters the
    projection in series with the robot yaw, so a frame grabbed mid-sweep
    carries an unknown bearing and its labels would be wrong by up to tens of
    centimetres -- silently, and in the training set.

NEGATIVE FRAMES ARE KEPT ON PURPOSE

A frame with no visible berry gets an EMPTY label file, not no label file. In
YOLO format those two are different: an empty file is "there is nothing here,
and I am sure", a missing file is "unlabelled, skip". Images of bare gutters,
leaves and glass are what the model learns "not a strawberry" from, and they
are the direct antidote to the false-positive rate that motivates this whole
exercise.

USAGE
    ros2 run youbot_slam dataset_capture --ros-args \\
      -p session:=twin_run_01 -p target_frames:=2000
Then annotate nothing -- it is already annotated -- and train.
"""

from __future__ import annotations

import csv
import json
import math
import os
from datetime import datetime

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64, String

from youbot_slam.lib.berry_view import CameraModel


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DatasetCapture(Node):
    def __init__(self) -> None:
        super().__init__("dataset_capture")

        self.declare_parameter("catalogue", "")      # berries.yaml; "" = find it
        self.declare_parameter("session", "")
        self.declare_parameter("out_dir", "")
        self.declare_parameter("image_topic", "camera/image")
        self.declare_parameter("target_frames", 2000)
        self.declare_parameter("min_translation", 0.20)   # m between frames
        self.declare_parameter("min_rotation_deg", 10.0)
        self.declare_parameter("keep_empty_frames", True)
        self.declare_parameter("empty_frame_ratio", 0.35)  # cap on negatives
        self.declare_parameter("require_pan_settled", True)
        # Camera model. Must match the URDF sensor block; check_regressions
        # cross-checks them so the labels cannot drift from the optics.
        self.declare_parameter("cam_hfov", 1.2)
        self.declare_parameter("cam_pitch", 0.28)
        self.declare_parameter("cam_range", 2.2)
        self.declare_parameter("cam_mount_x", 0.18)
        self.declare_parameter("cam_mount_y", 0.0)
        self.declare_parameter("cam_mount_z", 0.78)
        self.declare_parameter("cam_arm", 0.14)
        self.declare_parameter("min_blob_px", 2.5)
        self.declare_parameter("use_truth_pose", True)
        self.declare_parameter("truth_topic", "truth_pose")

        self._session = (str(self.get_parameter("session").value)
                         or datetime.now().strftime("twin_%Y%m%d_%H%M%S"))
        self._dir = (str(self.get_parameter("out_dir").value)
                     or os.path.expanduser(
                         f"~/youbot_datasets/{self._session}"))

        self._berries, self._foliage = self._load_catalogue()
        self._cam = None            # built on the first image, needs the size
        self._pose = None
        self._pan = None
        self._pan_settled = False
        self._last_kept = None
        self._saved = 0
        self._empty = 0
        self._seen = 0
        self._skipped_pan = 0
        self._csv = None
        self._writer = None
        self._label_hist: dict[int, int] = {}

        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value),
            self._on_image, 5)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter("truth_topic").value),
            self._on_truth, 10)
        self.create_subscription(Float64, "camera_pan_state", self._on_pan, 5)
        self.create_subscription(Bool, "camera_pan_settled",
                                 self._on_settled, 5)
        self.create_subscription(String, "/dataset/conditions",
                                 self._on_conditions, 1)
        self._conditions = ""
        self.create_timer(20.0, self._heartbeat)

        self.get_logger().info(
            f"dataset_capture up: {len(self._berries)} berries and "
            f"{len(self._foliage)} occluders in the catalogue -> {self._dir}")

    # ------------------------------------------------------------ inputs
    def _load_catalogue(self):
        path = str(self.get_parameter("catalogue").value)
        if not path:
            try:
                from ament_index_python.packages import get_package_share_directory
                path = os.path.join(
                    get_package_share_directory("youbot_gazebo"),
                    "worlds", "berries.yaml")
            except Exception:
                path = ""
        if not path or not os.path.exists(path):
            self.get_logger().error(
                "no berry catalogue found. Without ground truth this node has "
                "nothing to label with -- run make_world.py and pass "
                "catalogue:=<path>/berries.yaml")
            return [], []
        with open(path) as fh:
            data = yaml.safe_load(fh) or {}
        return (list(data.get("berries") or []),
                list(data.get("foliage") or []))

    def _on_odom(self, msg: Odometry) -> None:
        if bool(self.get_parameter("use_truth_pose").value):
            return
        p = msg.pose.pose
        self._pose = (p.position.x, p.position.y, _yaw(p.orientation))

    def _on_truth(self, msg: PoseStamped) -> None:
        if not bool(self.get_parameter("use_truth_pose").value):
            return
        # Truth, deliberately. A label placed with a drifting pose is a wrong
        # label; localisation error belongs in the localisation report, not
        # baked into the training set where nothing can ever remove it.
        self._pose = (msg.pose.position.x, msg.pose.position.y,
                      _yaw(msg.pose.orientation))

    def _on_pan(self, msg: Float64) -> None:
        self._pan = float(msg.data)

    def _on_settled(self, msg: Bool) -> None:
        self._pan_settled = bool(msg.data)

    def _on_conditions(self, msg: String) -> None:
        self._conditions = msg.data.strip()

    # ------------------------------------------------------------- logic
    def _ready(self) -> bool:
        if self._pose is None or not self._berries:
            return False
        if self._pan is None:
            return False
        if (bool(self.get_parameter("require_pan_settled").value)
                and not self._pan_settled):
            self._skipped_pan += 1
            return False
        return True

    def _moved_enough(self) -> bool:
        if self._last_kept is None:
            return True
        dx = self._pose[0] - self._last_kept[0]
        dy = self._pose[1] - self._last_kept[1]
        dyaw = abs(math.atan2(math.sin(self._pose[2] - self._last_kept[2]),
                              math.cos(self._pose[2] - self._last_kept[2])))
        return (math.hypot(dx, dy)
                >= float(self.get_parameter("min_translation").value)
                or dyaw >= math.radians(
                    float(self.get_parameter("min_rotation_deg").value)))

    def _on_image(self, msg: Image) -> None:
        self._seen += 1
        if self._saved >= int(self.get_parameter("target_frames").value):
            return
        if not self._ready() or not self._moved_enough():
            return
        rgb = self._to_rgb(msg)
        if rgb is None:
            return
        if self._cam is None:
            self._cam = self._build_camera(msg.width, msg.height)

        labels = self._cam.visible(self._pose, self._pan,
                                   self._berries, self._foliage)

        if not labels:
            if not bool(self.get_parameter("keep_empty_frames").value):
                return
            # Cap the negatives. A set that is 90% empty frames trains a model
            # whose safest answer is always "nothing here".
            cap = float(self.get_parameter("empty_frame_ratio").value)
            if self._saved and self._empty / max(1, self._saved) > cap:
                return
            self._empty += 1

        self._write(rgb, labels, msg)

    def _build_camera(self, width, height) -> CameraModel:
        g = self.get_parameter
        cam = CameraModel(
            width=width, height=height,
            hfov=float(g("cam_hfov").value),
            pitch=float(g("cam_pitch").value),
            max_range=float(g("cam_range").value),
            min_blob_px=float(g("min_blob_px").value),
            mount_x=float(g("cam_mount_x").value),
            mount_y=float(g("cam_mount_y").value),
            mount_z=float(g("cam_mount_z").value),
            arm=float(g("cam_arm").value))
        self.get_logger().info(
            f"camera model: {width}x{height}, fx={cam.fx:.1f} px/rad, "
            f"hfov={math.degrees(cam.hfov):.1f} deg, "
            f"vfov={math.degrees(cam.vfov):.1f} deg")
        return cam

    def _ensure_dirs(self) -> None:
        if self._csv is not None:
            return
        os.makedirs(os.path.join(self._dir, "images"), exist_ok=True)
        os.makedirs(os.path.join(self._dir, "labels"), exist_ok=True)
        self._csv = open(os.path.join(self._dir, "index.csv"), "w", newline="")
        self._writer = csv.writer(self._csv)
        self._writer.writerow(
            ["filename", "stamp_s", "x", "y", "yaw_rad", "pan_rad", "n_berries"])

    def _write(self, rgb, labels, msg) -> None:
        self._ensure_dirs()
        name = f"{self._saved:06d}"
        img_path = os.path.join(self._dir, "images", name + ".png")
        if not self._write_png(img_path, rgb):
            return
        # YOLO: one line per object, "class cx cy w h", all normalised.
        # An EMPTY file is a valid and meaningful label: "nothing here".
        with open(os.path.join(self._dir, "labels", name + ".txt"), "w") as fh:
            for lab in labels:
                cx, cy, w, h = lab["box"]
                cls = 0                      # single class until the world
                                             # generator records ripeness
                fh.write(f"{cls} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n")
                self._label_hist[cls] = self._label_hist.get(cls, 0) + 1
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._writer.writerow([name + ".png", f"{stamp:.6f}",
                               f"{self._pose[0]:.4f}", f"{self._pose[1]:.4f}",
                               f"{self._pose[2]:.5f}", f"{self._pan:.5f}",
                               len(labels)])
        self._csv.flush()
        self._last_kept = self._pose
        self._saved += 1

    @staticmethod
    def _to_rgb(msg: Image):
        try:
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding in ("rgb8", "bgr8"):
                img = buf.reshape(msg.height, msg.width, 3)
                return img[:, :, ::-1] if msg.encoding == "bgr8" else img
            if msg.encoding in ("rgba8", "bgra8"):
                img = buf.reshape(msg.height, msg.width, 4)[:, :, :3]
                return img[:, :, ::-1] if msg.encoding == "bgra8" else img
        except ValueError:
            return None
        return None

    @staticmethod
    def _write_png(path: str, rgb) -> bool:
        try:
            from PIL import Image as PILImage
            PILImage.fromarray(rgb).save(path)
            return True
        except ImportError:
            with open(path.rsplit(".", 1)[0] + ".ppm", "wb") as fh:
                fh.write(b"P6\n%d %d\n255\n" % (rgb.shape[1], rgb.shape[0]))
                fh.write(rgb.tobytes())
            return True
        except OSError:
            return False

    # -------------------------------------------------------- reporting
    def _heartbeat(self) -> None:
        target = int(self.get_parameter("target_frames").value)
        boxes = sum(self._label_hist.values())
        self.get_logger().info(
            f"[dataset] {self._saved}/{target} frames "
            f"({self._empty} empty, {boxes} boxes, {self._seen} seen, "
            f"{self._skipped_pan} dropped while the head moved)")
        if self._saved >= target:
            self._finish()

    def _finish(self) -> None:
        if self._csv is not None:
            try:
                self._csv.close()
            except OSError:
                pass
            self._csv = None
        if not self._saved:
            self.get_logger().warn("no frames captured; nothing written")
            return
        boxes = sum(self._label_hist.values())
        manifest = {
            "session": self._session,
            "conditions": self._conditions,
            "frames": self._saved,
            "empty_frames": self._empty,
            "boxes": boxes,
            "boxes_per_frame": boxes / max(1, self._saved),
            "frames_seen": self._seen,
            "frames_dropped_head_moving": self._skipped_pan,
            "catalogue_berries": len(self._berries),
            "catalogue_occluders": len(self._foliage),
            "label_source": "ground-truth catalogue via "
                            "youbot_slam.lib.berry_view (NOT the detector)",
            "pose_source": ("truth"
                            if bool(self.get_parameter("use_truth_pose").value)
                            else "odometry"),
            "format": "YOLO: class cx cy w h, normalised; empty file = no object",
            "created": datetime.now().isoformat(),
        }
        try:
            with open(os.path.join(self._dir, "dataset.json"), "w") as fh:
                json.dump(manifest, fh, indent=2)
            with open(os.path.join(self._dir, "classes.txt"), "w") as fh:
                fh.write("strawberry\n")
        except OSError as exc:
            self.get_logger().warn(f"could not write the manifest: {exc}")
        self.get_logger().info(
            f"--- dataset complete: {self._saved} frames, {boxes} boxes, "
            f"{self._empty} negatives, at {self._dir}")
        self.get_logger().info(
            "Labels came from ground truth, not from the colour detector, so "
            "training on them cannot reproduce its false positives.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DatasetCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("interrupted -- writing what was captured")
        node._finish()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
