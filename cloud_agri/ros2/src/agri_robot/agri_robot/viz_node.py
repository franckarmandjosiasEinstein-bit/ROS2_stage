"""viz_node -- what the robot sees, in 2D, next to what Gazebo renders.

Gazebo shows the greenhouse. This shows the LIDAR's greenhouse: the outline
the robot actually perceives, the crosses it is aiming at, and the corridor
its brake is watching. When the robot stops for no visible reason, this is
where the reason is.

It publishes three things and nothing else:

    TF map -> base_link   from /odom, so RViz can place the scan at all.
                          Without it every display is greyed out and RViz
                          says only "no transform from [lidar] to [map]",
                          which is a true statement about a missing node
                          rather than about the robot.
    /agri/stations        the 48 red crosses, the walls and gutters as the
                          CATALOGUE believes them to be, and the robot's own
                          outline including its sensor boom. The comparison
                          is the whole value of the view: a lidar scan on its
                          own is a pretty shape with no scale, and the moment
                          it stops lying on top of these lines, either the
                          odometry or the catalogue is wrong.

WHY NOT REUSE PHASE B's odom_tf

It publishes map -> base_link from /odom too, and it would work. It lives in
youbot_control, which would make this package depend on the whole harvesting
stack -- its parameters, its launch assumptions, its future changes -- for
eleven lines of quaternion. The two projects share a greenhouse and a robot
mesh; they should not share a build graph.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from agri.aisles import BODY_BOX, GUTTER_HALF_LENGTH
from agri.catalogue import (CROSS_ARM, GUTTER_HALF_WIDTH, ROW_Y, WALL_X,
                            WALL_Y, all_stations)

FRAME = "map"

#: The world markers are static, so they are published TRANSIENT_LOCAL: a
#: subscriber that connects late still gets the last message instead of
#: waiting for the next one. config/agri.rviz asks for the same durability,
#: and it has to -- a Transient Local subscriber and a Volatile publisher are
#: INCOMPATIBLE in DDS, so RViz would receive nothing at all and the display
#: would look exactly like a node that is not running.
WORLD_QOS = QoSProfile(depth=1,
                       history=HistoryPolicy.KEEP_LAST,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _marker(mid: int, kind: int, scale, rgba, ns: str = "agri") -> Marker:
    m = Marker()
    m.header.frame_id = FRAME
    m.ns, m.id, m.type, m.action = ns, mid, kind, Marker.ADD
    m.scale.x, m.scale.y, m.scale.z = scale
    m.color.r, m.color.g, m.color.b, m.color.a = rgba
    m.pose.orientation.w = 1.0
    return m


class Viz(Node):
    def __init__(self) -> None:
        super().__init__("agri_viz")
        self.tf = TransformBroadcaster(self)
        self.stations = self.create_publisher(MarkerArray, "agri/stations",
                                              WORLD_QOS)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        # Latched-ish: the world does not move, but RViz may start late, so
        # republish slowly forever rather than once and hope.
        self.create_timer(2.0, self._publish_world)
        self._publish_world()

    def _on_odom(self, msg: Odometry) -> None:
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = FRAME
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(t)

    # ------------------------------------------------------------- markers
    def _publish_world(self) -> None:
        arr = MarkerArray()
        arr.markers.append(self._crosses())
        arr.markers.append(self._structure())
        arr.markers.append(self._footprint())
        self.stations.publish(arr)

    def _crosses(self) -> Marker:
        """The 48 stations, drawn as the crosses that are painted there."""
        m = _marker(0, Marker.LINE_LIST, (0.012, 0.0, 0.0), (0.95, 0.1, 0.1, 1.0))
        for s in all_stations():
            m.points.extend(_seg(s.x - CROSS_ARM, s.y, s.x + CROSS_ARM, s.y))
            m.points.extend(_seg(s.x, s.y - CROSS_ARM, s.x, s.y + CROSS_ARM))
        return m

    def _structure(self) -> Marker:
        """Walls and gutters, from the catalogue -- NOT from the world file.

        Drawn from the same numbers the robot navigates by, on purpose. If
        the lidar's outline and these lines disagree, the catalogue is wrong
        about the greenhouse, and that is exactly the failure this view
        should make obvious rather than hide.
        """
        m = _marker(1, Marker.LINE_LIST, (0.02, 0.0, 0.0), (0.3, 0.7, 0.9, 0.8))
        m.points.extend(_box(-WALL_X, -WALL_Y, WALL_X, WALL_Y))
        for y in ROW_Y.values():
            m.points.extend(_box(-GUTTER_HALF_LENGTH, y - GUTTER_HALF_WIDTH,
                                 +GUTTER_HALF_LENGTH, y + GUTTER_HALF_WIDTH))
        return m

    def _footprint(self) -> Marker:
        """The robot's own outline, in ITS frame, including the sensor boom.

        In base_link rather than map, so it follows the robot for free, and
        so what you see is the rectangle the brake is reasoning about rather
        than an artist's impression of a youbot.
        """
        m = _marker(2, Marker.LINE_LIST, (0.02, 0.0, 0.0), (1.0, 0.7, 0.1, 0.9))
        m.header.frame_id = "base_link"
        m.points.extend(_box(*BODY_BOX))
        return m


def _pt(x: float, y: float, z: float = 0.01):
    from geometry_msgs.msg import Point                # noqa: PLC0415
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


def _seg(x0, y0, x1, y1, z=0.01):
    return [_pt(x0, y0, z), _pt(x1, y1, z)]


def _box(x0, y0, x1, y1, z=0.01):
    return (_seg(x0, y0, x1, y0, z) + _seg(x1, y0, x1, y1, z)
            + _seg(x1, y1, x0, y1, z) + _seg(x0, y1, x0, y0, z))


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = Viz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
