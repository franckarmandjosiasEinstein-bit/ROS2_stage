"""sim_node -- headless stand-in for Webots, so the whole stack runs on ROS alone.

Integrates /cmd_vel through the mecanum body kinematics to publish /odom, and
ray-casts a synthetic /scan against a known obstacle layout (the smart-
agriculture bassins + arena walls). It also broadcasts TF (map -> base_link ->
lidar) so RViz can render the robot and its scan.

No physics, no collision: the robot is a point that moves exactly as commanded
-- ideal for developing and debugging the control nodes without a simulator.

    ros2 launch youbot_bringup sim.launch.py
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped, PoseArray, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

# --- Strawberry greenhouse (from the real plan) -----------------------------
# Footprint 10 m (X, length) x 5 m (Y, width); origin at centre. Three culture
# gutters run along the length (8.0 m x 0.4 m) at Y = -1.2, 0, +1.2, leaving
# 0.8 m driving aisles between them and ~1.1 m margins to the side walls. Open
# cross-corridors at both X ends (gutters span X in [-4, 4]).
ARENA_X_HALF = 5.0   # walls at X = +/- 5 m  (10 m long)
ARENA_Y_HALF = 2.5   # walls at Y = +/- 2.5 m (5 m wide)

# Obstacles the lidar sees: (centre_x, centre_y, size_x, size_y) -- the gutters.
OBSTACLES = [
    (0.0, 1.2, 8.0, 0.4),
    (0.0, 0.0, 8.0, 0.4),
    (0.0, -1.2, 8.0, 0.4),
]

# Crates to collect, placed in the 0.8 m aisles (Y = +/- 0.6) next to a gutter.
CRATES = [(-2.0, 0.6), (2.5, -0.6), (0.5, 0.6)]
CAM_FOV = 2.0      # rad: wide cone so the patrol reliably spots crates in sim
CAM_RANGE = 4.0    # m: only "detect" a crate within this range


def _yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class SimNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_node")
        self.declare_parameter("x", -4.6)   # left corridor, top margin, facing +X
        self.declare_parameter("y", 1.9)
        self.declare_parameter("theta", 0.0)
        self.declare_parameter("n_beams", 360)
        self.declare_parameter("max_range", 12.0)

        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.th = float(self.get_parameter("theta").value)
        self.n = int(self.get_parameter("n_beams").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.vx = self.vy = self.wz = 0.0

        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.scan_pub = self.create_publisher(LaserScan, "scan", 10)
        self.crate_pub = self.create_publisher(MarkerArray, "crate_markers", 1)
        self.detect_pub = self.create_publisher(PoseArray, "detected_crates", 5)
        self.tf = TransformBroadcaster(self)
        self._static_tf()

        self.dt = 0.05
        self.create_timer(self.dt, self._tick)             # 20 Hz motion + odom
        self.create_timer(0.1, self._publish_scan)         # 10 Hz lidar
        self.create_timer(1.0, self._publish_crate_markers)  # crates in RViz
        self.create_timer(0.2, self._publish_detections)   # camera-cone "vision"
        self.get_logger().info(
            "sim_node up: fake robot (no Webots). /cmd_vel -> /odom + /scan + TF; "
            "crates -> /crate_markers + /detected_crates (FOV cone)")

    def _static_tf(self) -> None:
        st = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "lidar"
        t.transform.rotation.w = 1.0
        st.sendTransform(t)

    def _on_cmd(self, msg: Twist) -> None:
        self.vx, self.vy, self.wz = msg.linear.x, msg.linear.y, msg.angular.z

    def _tick(self) -> None:
        c, s = math.cos(self.th), math.sin(self.th)
        self.x += (c * self.vx - s * self.vy) * self.dt
        self.y += (s * self.vx + c * self.vy) * self.dt
        self.th = math.atan2(math.sin(self.th + self.wz * self.dt),
                             math.cos(self.th + self.wz * self.dt))
        qz, qw = _yaw_to_quat(self.th)

        now = self.get_clock().now().to_msg()
        o = Odometry()
        o.header.stamp = now
        o.header.frame_id = "map"
        o.child_frame_id = "base_link"
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        self.odom_pub.publish(o)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "map"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf.sendTransform(tf)

    def _publish_scan(self) -> None:
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "lidar"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / self.n
        scan.range_min = 0.05
        scan.range_max = self.max_range
        ranges = []
        for i in range(self.n):
            a = self.th + scan.angle_min + i * scan.angle_increment
            ranges.append(self._raycast(a))
        scan.ranges = ranges
        self.scan_pub.publish(scan)

    def _publish_crate_markers(self) -> None:
        arr = MarkerArray()
        for i, (cx, cy) in enumerate(CRATES):
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
            m.color.r = 0.9
            m.color.g = 0.1
            m.color.b = 0.1
            m.color.a = 1.0
            arr.markers.append(m)
        self.crate_pub.publish(arr)

    def _publish_detections(self) -> None:
        """Emulate the camera: report crates inside the front FOV cone/range."""
        pa = PoseArray()
        pa.header.frame_id = "map"
        pa.header.stamp = self.get_clock().now().to_msg()
        for cx, cy in CRATES:
            dx, dy = cx - self.x, cy - self.y
            dist = math.hypot(dx, dy)
            if dist > CAM_RANGE:
                continue
            bearing = math.atan2(math.sin(math.atan2(dy, dx) - self.th),
                                 math.cos(math.atan2(dy, dx) - self.th))
            if abs(bearing) > CAM_FOV / 2.0:
                continue
            p = Pose()
            p.position.x = cx
            p.position.y = cy
            p.orientation.w = 1.0
            pa.poses.append(p)
        self.detect_pub.publish(pa)

    def _raycast(self, ang: float) -> float:
        dx, dy = math.cos(ang), math.sin(ang)
        # Distance to the rectangular greenhouse walls in this direction.
        tx = ((ARENA_X_HALF if dx > 0 else -ARENA_X_HALF) - self.x) / dx if abs(dx) > 1e-9 else math.inf
        ty = ((ARENA_Y_HALF if dy > 0 else -ARENA_Y_HALF) - self.y) / dy if abs(dy) > 1e-9 else math.inf
        best = min(tx, ty)
        for cx, cy, sx, sy in OBSTACLES:
            t = self._ray_box(self.x, self.y, dx, dy, cx, cy, sx, sy)
            if t is not None and 0.0 < t < best:
                best = t
        return best if best < self.max_range else float("inf")

    @staticmethod
    def _ray_box(px, py, dx, dy, cx, cy, sx, sy):
        x0, x1 = cx - sx / 2.0, cx + sx / 2.0
        y0, y1 = cy - sy / 2.0, cy + sy / 2.0
        tmin, tmax = -math.inf, math.inf
        for p, d, lo, hi in ((px, dx, x0, x1), (py, dy, y0, y1)):
            if abs(d) < 1e-9:
                if p < lo or p > hi:
                    return None
            else:
                ta, tb = (lo - p) / d, (hi - p) / d
                if ta > tb:
                    ta, tb = tb, ta
                tmin = max(tmin, ta)
                tmax = min(tmax, tb)
        if tmax < max(tmin, 0.0):
            return None
        return tmin if tmin > 0.0 else None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimNode()
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
