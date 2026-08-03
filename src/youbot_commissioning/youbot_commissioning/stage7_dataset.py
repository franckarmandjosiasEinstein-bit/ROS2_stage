"""Stage 7 -- collect the real image dataset, with the robot navigating.

This is the stage that unblocks everything downstream, and it is deliberately
the dumbest node in the package: it saves frames and it saves the pose that
went with them. Nothing is detected, nothing is judged.

WHY THE POSE IS SAVED WITH EVERY FRAME
Two reasons, and the second is the one people forget.

  1. It lets a later detection be back-projected into the map, so the
     annotated dataset can be turned into ground-truth fruit POSITIONS and
     not merely fruit boxes.
  2. It lets the dataset be de-duplicated honestly. A camera at 10 Hz on a
     robot moving at 0.2 m/s produces twenty near-identical pictures of the
     same plant. Training on all twenty and testing on the twenty-first is
     how a model scores 99% and then fails in the field. This node keeps a
     frame only when the robot has MOVED far enough or TURNED enough since
     the last one, so consecutive frames are genuinely different views.

WHAT TO COLLECT, AS A PROTOCOL
Coverage of nuisance conditions matters more than raw count. 500 images
spanning the day beat 5000 taken in one hour:
  * morning, midday, late afternoon, and overcast if possible
  * both sides of the aisle (sunlit row and shaded row)
  * ripe, half-ripe and green fruit, deliberately including the ambiguous ones
  * fruit partly hidden by leaves -- this is the majority case in reality
  * frames containing red objects that are NOT fruit: pipes, tools, crates,
    clothing. Negative examples are not optional; they are what the model
    learns "not strawberry" from.
"""

from __future__ import annotations

import csv
import json
import math
import os
from datetime import datetime

import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from youbot_commissioning.lib.report import results_dir
from youbot_commissioning.lib.stage import CommissioningStage, run


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Stage7(CommissioningStage):
    STAGE = 7
    SLUG = "dataset"
    TITLE = "Real image dataset collection"
    PROCEDURE = """
    BEFORE ARMING
      1. Stage 6 has PASSED, so the robot can drive the aisles safely.
      2. Decide the session label and set it, because you will run this many
         times and an unlabelled session is a lost session:
           ros2 topic pub -1 /commissioning/session std_msgs/String \\
             "{data: '2026-04-12_morning_sunny'}"
      3. Check there is disk space. 2000 frames at 640x480 is about 1 GB.

    WHAT WILL HAPPEN
      The robot drives its normal survey route. This node saves a frame each
      time the robot has moved far enough since the last one, together with
      the pose. It writes a PNG per frame plus an index.csv and a
      session.json describing the conditions.

    ALSO RECORD THE CONDITIONS -- they are half the value of the dataset
      ros2 topic pub -1 /commissioning/conditions std_msgs/String \\
        "{data: 'overcast, 11h20, shade on the north row, irrigation running'}"

    WHEN DONE
      Ctrl-C. Then ANNOTATE. Without annotations this is a pile of pictures.
      Use CVAT, Label Studio or Roboflow; export YOLO format next to the
      images.

    PASS MEANS
      Enough frames, spread over enough distinct poses, with the session and
      conditions recorded. This stage cannot tell you the dataset is GOOD --
      only stage 8 can, and only after annotation.
    """

    def __init__(self):
        super().__init__("stage7_dataset")
        self.declare_parameter("image_topic", "camera/image")
        self.declare_parameter("info_topic", "camera/camera_info")
        self.declare_parameter("min_translation", 0.25)   # m between frames
        self.declare_parameter("min_rotation_deg", 12.0)  # or this much turn
        self.declare_parameter("target_frames", 500)
        self.declare_parameter("min_distinct_poses", 100)

        # This stage does not drive either: the mission stack does.
        self._vmax = 0.0
        self._wmax = 0.0

        self._session = datetime.now().strftime("session_%Y%m%d_%H%M%S")
        self._conditions = ""
        self._dir = None
        self._csv = None
        self._writer = None
        self._pose = None
        self._last_kept = None
        self._saved = 0
        self._seen = 0
        self._info = None

        self.create_subscription(Image,
                                 str(self.get_parameter("image_topic").value),
                                 self._on_image, 5)
        self.create_subscription(CameraInfo,
                                 str(self.get_parameter("info_topic").value),
                                 self._on_info, 1)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(String, "/commissioning/session",
                                 self._on_session, 1)
        self.create_subscription(String, "/commissioning/conditions",
                                 self._on_conditions, 1)
        self.create_timer(30.0, self._heartbeat)

    # --- session bookkeeping -------------------------------------------
    def _on_session(self, msg: String) -> None:
        if self._saved:
            self.get_logger().warn("session already started; label ignored")
            return
        self._session = msg.data.strip().replace(" ", "_")
        self.get_logger().info(f"session label set to '{self._session}'")

    def _on_conditions(self, msg: String) -> None:
        self._conditions = msg.data.strip()
        self.report.note(f"conditions: {self._conditions}")

    def _on_info(self, msg: CameraInfo) -> None:
        if self._info is None:
            self._info = {"width": msg.width, "height": msg.height,
                          "k": list(msg.k), "d": list(msg.d),
                          "frame_id": msg.header.frame_id}

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._pose = (p.position.x, p.position.y, _yaw(p.orientation))

    def _ensure_dir(self) -> None:
        if self._dir is not None:
            return
        self._dir = os.path.join(results_dir(), "datasets", self._session)
        os.makedirs(os.path.join(self._dir, "images"), exist_ok=True)
        self._csv = open(os.path.join(self._dir, "index.csv"), "w", newline="")
        self._writer = csv.writer(self._csv)
        self._writer.writerow(["filename", "stamp_s", "x", "y", "yaw_rad"])
        self.get_logger().info(f"writing dataset to {self._dir}")

    # --- the actual work -------------------------------------------------
    def _far_enough(self) -> bool:
        if self._pose is None:
            return False
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
        if not self._far_enough():
            return
        rgb = self._to_rgb(msg)
        if rgb is None:
            return
        self._ensure_dir()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        name = f"{self._saved:05d}.png"
        path = os.path.join(self._dir, "images", name)
        if not self._write_png(path, rgb):
            return
        self._writer.writerow([name, f"{stamp:.6f}",
                               f"{self._pose[0]:.4f}", f"{self._pose[1]:.4f}",
                               f"{self._pose[2]:.5f}"])
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
        """PNG without pulling in OpenCV: Pillow if present, else PPM."""
        try:
            from PIL import Image as PILImage
            PILImage.fromarray(rgb).save(path)
            return True
        except ImportError:
            ppm = path.rsplit(".", 1)[0] + ".ppm"
            with open(ppm, "wb") as fh:
                fh.write(b"P6\n%d %d\n255\n" % (rgb.shape[1], rgb.shape[0]))
                fh.write(rgb.tobytes())
            return True
        except OSError:
            return False

    def _heartbeat(self) -> None:
        target = int(self.get_parameter("target_frames").value)
        self.get_logger().info(
            f"[stage7] {self._saved}/{target} frames kept "
            f"({self._seen} seen) -- session '{self._session}'")

    def stop(self) -> None:
        super().stop()
        if self._csv is not None:
            try:
                self._csv.close()
            except OSError:
                pass
            self._csv = None
        if self._dir and not self.report.checks:
            self._write_manifest()
            self._conclude()

    def _write_manifest(self) -> None:
        manifest = {
            "session": self._session,
            "conditions": self._conditions,
            "frames": self._saved,
            "frames_seen": self._seen,
            "camera_info": self._info,
            "min_translation_m":
                float(self.get_parameter("min_translation").value),
            "min_rotation_deg":
                float(self.get_parameter("min_rotation_deg").value),
            "created": datetime.now().isoformat(),
            "annotation_status": "NOT ANNOTATED",
        }
        try:
            with open(os.path.join(self._dir, "session.json"), "w") as fh:
                json.dump(manifest, fh, indent=2)
        except OSError as exc:
            self.get_logger().warn(f"could not write session.json: {exc}")

    def _conclude(self) -> None:
        self.report.record("session", self._session)
        self.report.record("directory", self._dir)
        self.report.record("frames_kept", self._saved)
        self.report.record("frames_seen", self._seen)
        self.report.record("conditions", self._conditions)

        self.report.check("frames collected", self._saved, ">=",
                          int(self.get_parameter("target_frames").value))
        self.report.check("distinct poses", self._saved, ">=",
                          int(self.get_parameter("min_distinct_poses").value),
                          "", "each kept frame is at least 0.25 m or 12 deg "
                              "from the previous one")
        self.report.check("conditions recorded",
                          1 if self._conditions else 0, "==", 1,
                          note="an unlabelled session cannot be used to "
                               "explain why the model fails at 17h")
        self.report.note(
            "NEXT: annotate this session (CVAT / Label Studio / Roboflow), "
            "export YOLO format, then run stage 8 against it. Until then the "
            "dataset has no measurable value.")


def main(args=None) -> None:
    run(Stage7)


if __name__ == "__main__":
    main()
