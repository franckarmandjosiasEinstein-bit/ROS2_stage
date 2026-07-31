"""odom_calibrator -- learn the odometry's systematic bias, then run without help.

Why this exists. In a straight aisle between two parallel gutters, the lidar
constrains Y and heading perfectly but says almost nothing about X (sliding
along a uniform corridor does not change what you see -- the aperture
problem). Along X the SLAM therefore rides on odometry alone, so odometry's
systematic scale bias integrates straight into the pose: at +4% over an 8 m
aisle that is 0.32 m per pass, ~0.9 m after three passes -- exactly the
residual plateau measured in the field log.

Scan matching cannot fix that (it has no X information to fix it with), but
CALIBRATION can: a scale bias is a constant of the machine (gear ratio, wheel
diameter, roller geometry), so it can be measured once and compensated
forever. That is standard practice on real robots (cf. UMBmark) -- the
ground-truth reference here plays the role of the tape measure on the floor.

Two modes:

    calibrate: /odom (reference) + /odom_noisy -> least-squares scale factors,
               refined continuously, applied live to /odom_calibrated, and
               written to a YAML file once converged.
    apply:     no reference at all -- read the saved factors and correct
               /odom_noisy into /odom_calibrated. This is the real-robot mode.

Least squares on body-frame increments: with noisy = truth * (1 + b) + noise,
    scale = sum(noisy * truth) / sum(truth * truth)
is unbiased for zero-mean noise and converges in a few metres of driving.

Subscribes:  /odom_noisy (nav_msgs/Odometry)
             /odom       (nav_msgs/Odometry)  reference, CALIBRATE MODE ONLY
Publishes:   /odom_calibrated (nav_msgs/Odometry)  bias-compensated odometry
"""

from __future__ import annotations

import math
import os

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


# Minimum accumulated squared motion per axis before its scale is trusted
# (x, y, yaw). Below this the axis is not excited enough to be identifiable.
MIN_EXCITATION = (0.015, 0.015, 0.008)


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def body_increment(prev, cur):
    """Displacement from prev to cur, expressed in prev's body frame."""
    dxw, dyw = cur[0] - prev[0], cur[1] - prev[1]
    c, s = math.cos(-prev[2]), math.sin(-prev[2])
    return (c * dxw - s * dyw,
            s * dxw + c * dyw,
            math.atan2(math.sin(cur[2] - prev[2]), math.cos(cur[2] - prev[2])))


class OdomCalibrator(Node):
    def __init__(self) -> None:
        super().__init__("odom_calibrator")
        self.declare_parameter("mode", "calibrate")     # calibrate | apply
        self.declare_parameter("calib_file",
                               os.path.expanduser("~/.ros/youbot_odom_calib.yaml"))
        # Distance to drive before the estimate is trusted and saved.
        self.declare_parameter("min_distance", 3.0)     # m
        self.declare_parameter("report_period", 10.0)   # s

        self._mode = str(self.get_parameter("mode").value)
        self._file = str(self.get_parameter("calib_file").value)
        self._min_dist = float(self.get_parameter("min_distance").value)

        # Least-squares accumulators: sum(noisy*truth) and sum(truth*truth).
        self._nt = [0.0, 0.0, 0.0]
        self._tt = [0.0, 0.0, 0.0]
        self._dist = 0.0                 # reference distance travelled
        self._scale = [1.0, 1.0, 1.0]    # sx, sy, syaw
        self._saved = False

        self._prev_truth = None
        self._prev_noisy = None
        self._truth = {}                 # stamp -> (x, y, yaw)
        self._pose = None                # integrated calibrated pose

        if self._mode == "apply":
            self._load()

        self.pub = self.create_publisher(Odometry, "odom_calibrated", 20)
        self.create_subscription(Odometry, "odom_noisy", self._on_noisy, 20)
        if self._mode == "calibrate":
            self.create_subscription(Odometry, "odom", self._on_truth, 20)
        self.create_timer(float(self.get_parameter("report_period").value), self._report)
        self.get_logger().info(
            f"odom_calibrator up [{self._mode}]: /odom_noisy -> /odom_calibrated"
            + (f" (reference /odom, saving to {self._file})"
               if self._mode == "calibrate" else
               f" (scales {self._scale[0]:.4f} / {self._scale[1]:.4f} / {self._scale[2]:.4f})"))

    # ------------------------------------------------------------- reference
    def _on_truth(self, msg: Odometry) -> None:
        p = msg.pose.pose
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self._truth[key] = (p.position.x, p.position.y,
                            yaw_from_quaternion(p.orientation))
        if len(self._truth) > 400:                    # bounded buffer
            for k in sorted(self._truth)[:200]:
                del self._truth[k]

    # ------------------------------------------------------------ estimation
    def _accumulate(self, key, noisy) -> None:
        truth = self._truth.pop(key, None)
        if truth is None:
            return
        if self._prev_truth is not None:
            t = body_increment(self._prev_truth, truth)
            n = body_increment(self._prev_noisy, noisy)
            for i in range(3):
                self._nt[i] += n[i] * t[i]
                self._tt[i] += t[i] * t[i]
            self._dist += math.hypot(t[0], t[1])
            for i in range(3):
                self._update_scale(i)
        self._prev_truth, self._prev_noisy = truth, noisy
        if not self._saved and self._dist >= self._min_dist:
            self._save()

    def _update_scale(self, i: int) -> None:
        """Least-squares scale, guarded against unexcited axes.

        An axis can only be calibrated if the robot actually MOVES along it
        (persistent excitation). This patrol drives forward and creeps
        forward/back to centre fruit, so the Y axis is barely excited -- its
        estimate would be noise. Leaving it at 1.0 is correct AND harmless:
        a scale error on a displacement that never happens costs nothing."""
        if self._tt[i] < MIN_EXCITATION[i]:
            return
        s = self._nt[i] / self._tt[i]
        self._scale[i] = min(1.30, max(0.77, s))     # reject absurd fits

    # --------------------------------------------------------------- output
    def _on_noisy(self, msg: Odometry) -> None:
        p = msg.pose.pose
        noisy = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

        if self._mode == "calibrate":
            self._accumulate((msg.header.stamp.sec, msg.header.stamp.nanosec), noisy)

        if self._pose is None:
            self._pose = noisy
            self._last_noisy = noisy
        else:
            # Undo the estimated scale on each body-frame increment, then
            # re-integrate: this is the corrected odometry the SLAM consumes.
            n = body_increment(self._last_noisy, noisy)
            self._last_noisy = noisy
            dxb = n[0] / self._scale[0] if abs(self._scale[0]) > 1e-3 else n[0]
            dyb = n[1] / self._scale[1] if abs(self._scale[1]) > 1e-3 else n[1]
            dth = n[2] / self._scale[2] if abs(self._scale[2]) > 1e-3 else n[2]
            x, y, yaw = self._pose
            c, s = math.cos(yaw), math.sin(yaw)
            self._pose = (x + c * dxb - s * dyb,
                          y + s * dxb + c * dyb,
                          math.atan2(math.sin(yaw + dth), math.cos(yaw + dth)))

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id or "base_link"
        out.pose.pose.position.x = self._pose[0]
        out.pose.pose.position.y = self._pose[1]
        out.pose.pose.position.z = p.position.z
        half = self._pose[2] / 2.0
        out.pose.pose.orientation.z = math.sin(half)
        out.pose.pose.orientation.w = math.cos(half)
        out.twist = msg.twist
        self.pub.publish(out)

    # ------------------------------------------------------------ persistence
    def _save(self) -> None:
        try:
            os.makedirs(os.path.dirname(self._file), exist_ok=True)
            with open(self._file, "w") as f:
                f.write("# Systematic odometry scale bias, measured by "
                        "youbot_slam/odom_calibrator.\n")
                f.write("# Apply with: mode:=apply (no ground-truth reference "
                        "needed any more).\n")
                f.write(f"scale_x: {self._scale[0]:.6f}\n")
                f.write(f"scale_y: {self._scale[1]:.6f}\n")
                f.write(f"scale_yaw: {self._scale[2]:.6f}\n")
            self._saved = True
            self.get_logger().info(
                f"Calibration converged after {self._dist:.1f} m and saved to "
                f"{self._file}: bias x {self._scale[0] - 1:+.1%}, "
                f"y {self._scale[1] - 1:+.1%}, yaw {self._scale[2] - 1:+.1%}. "
                "Re-run with mode:=apply to drop the reference.")
        except OSError as exc:
            self.get_logger().warn(f"Could not write {self._file}: {exc}")

    def _load(self) -> None:
        try:
            with open(self._file) as f:
                for line in f:
                    if line.startswith("#") or ":" not in line:
                        continue
                    key, val = line.split(":", 1)
                    idx = {"scale_x": 0, "scale_y": 1, "scale_yaw": 2}.get(key.strip())
                    if idx is not None:
                        self._scale[idx] = float(val)
        except (OSError, ValueError):
            self.get_logger().warn(
                f"No usable calibration in {self._file} -- passing odometry "
                "through unchanged. Run once with mode:=calibrate first.")

    def _report(self) -> None:
        if self._mode != "calibrate" or self._dist < 0.5:
            return
        self.get_logger().info(
            f"odometry bias estimate after {self._dist:.1f} m: "
            f"x {self._scale[0] - 1:+.1%}, y {self._scale[1] - 1:+.1%}, "
            f"yaw {self._scale[2] - 1:+.1%}"
            + ("" if self._saved else
               f" (refining -- {self._min_dist - self._dist:.1f} m to go; "
               "the robot only drives ~1.5 m/min while harvesting)"))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomCalibrator()
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
