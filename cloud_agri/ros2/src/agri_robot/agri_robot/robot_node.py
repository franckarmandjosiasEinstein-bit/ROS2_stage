"""robot_node -- the mobile node: ROS 2 on one side, the Cloud on the other.

    ros2 run agri_robot robot_node --ros-args -p broker:=localhost

WHAT IT IS

Three things bolted together, and the bolts are the interesting part:

    GazeboDriver    the body (driver.py): wheels, cameras, the floor marker
    Visitor/RobotLink  the brain (agri.robot): the same objects the offline
                    demo runs, unchanged
    paho.mqtt       the mouth: publish/subscribe on agri/v1/*

Nothing in agri.robot knows this file exists, and nothing in this file
re-implements a measurement, a QR code or an envelope. If the two ever
disagree about what a report looks like, the test suite fails offline, on a
machine with no simulator, in a second.

THREADS

rclpy spins on the main thread. A mission -- drive, wait, photograph, wait --
blocks for minutes, so it runs on a worker thread and the callbacks keep
filling in odometry and images underneath it. That is the only concurrency
here, and the only shared state is behind one lock in the driver.

ABOUT THE SENSOR READINGS

There is no humidity probe in Gazebo. The temperature, humidity, luminosity,
CO2 and pH are synthesised by agri.sensors from the station's position, with
a fixed seed, gradients across the greenhouse and a few deliberate anomalies.
This is stated at startup, in the logs, and in the README, because a number
that looks like a measurement and is not one is the single most misleading
thing a system like this can produce. Everything AROUND the number -- when it
was taken, where the robot was, what it photographed, how it was sealed, who
verified it -- is real.
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from agri import keys
from agri.protocol import (DEFAULT_BROKER, DEFAULT_PORT, QOS, TOPIC_REQUEST,
                           TOPIC_STATUS, mqtt_client)
from agri.robot import Mission, RobotLink, Visitor
from agri.sensors import GreenhouseField
from agri_robot.driver import DriveError, GazeboDriver


class AgriRobot(Node):
    def __init__(self) -> None:
        super().__init__("agri_robot")
        self.declare_parameter("robot_id", "youbot-01")
        self.declare_parameter("broker", DEFAULT_BROKER)
        self.declare_parameter("broker_port", DEFAULT_PORT)
        self.declare_parameter("keys", "keys")
        self.declare_parameter("status_period", 5.0)
        # Drive a list of stations with no Cloud at all. For bringing the
        # simulator up and watching the robot find its crosses before any
        # broker exists -- the reports are built and thrown away.
        self.declare_parameter("standalone_targets", "")

        def p(name):
            return self.get_parameter(name).value

        self.robot_id = p("robot_id")
        key_dir = Path(p("keys"))
        self.robot_keys = keys.ensure("robot", key_dir)
        self.cloud_public = keys.public_of("cloud", key_dir)

        field = GreenhouseField()
        self.driver = GazeboDriver(self, sensors=field.read,
                                   log=self.get_logger().info)
        self.visitor = Visitor(robot_id=self.robot_id,
                               cloud_public_pem=self.cloud_public,
                               robot_private_pem=self.robot_keys.private_pem,
                               read_sensors=field.read,
                               driver=self.driver)

        self.get_logger().info(
            f"agri_robot {self.robot_id}: keys in {key_dir.resolve()}, "
            "readings are SYNTHESISED by agri.sensors (Gazebo has no probes)")

        self.client = None
        self.link: RobotLink | None = None
        self._busy = threading.Lock()
        self.standalone = [t for t in str(p("standalone_targets")).split(";")
                           if t.strip()]

    # ------------------------------------------------------------- startup
    def connect(self) -> bool:
        host = self.get_parameter("broker").value
        port = int(self.get_parameter("broker_port").value)
        self.client = mqtt_client(f"agri-{self.robot_id}")
        self.link = RobotLink(self.visitor, self.cloud_public, self.client,
                              log=self.get_logger().info)

        # Last will: if this process dies, the broker tells the Cloud. A
        # dashboard that shows a robot as online because nobody said
        # otherwise is worse than one that shows nothing.
        self.client.will_set(TOPIC_STATUS, json.dumps({
            "schema": "agri-cloud/v1", "kind": "status",
            "robot": self.robot_id, "online": False,
            "note": "connection lost", "at": ""}), qos=QOS, retain=True)

        # BOTH callbacks are wrapped. paho does not guard them: an exception
        # raised inside one unwinds its network thread, and from then on the
        # client publishes (from other threads) but never READS. The robot
        # looks alive on the broker and ignores every order. Nothing in the
        # log says so, which is what makes it worth a decorator.
        def guarded(fn):
            def wrapper(*a, **kw):
                try:
                    return fn(*a, **kw)
                except Exception as exc:             # noqa: BLE001
                    self.get_logger().error(
                        f"{fn.__name__} raised {type(exc).__name__}: {exc} "
                        "-- swallowed, because letting it out would stop this "
                        "node hearing the Cloud")
            return wrapper

        @guarded
        def on_connect(cl, _u, _f, rc, *_):
            if rc:
                self.get_logger().error(f"broker refused the connection (rc={rc})")
                return
            cl.subscribe(TOPIC_REQUEST, qos=QOS)
            self.get_logger().info(
                f"connected to {host}:{port}, listening on {TOPIC_REQUEST}")
            self.link.status(True, "ready")

        @guarded
        def on_message(_cl, _u, msg):
            if msg.topic == TOPIC_REQUEST:
                self._take(msg.payload)

        self.client.on_connect, self.client.on_message = on_connect, on_message
        try:
            self.client.connect(host, port, keepalive=30)
        except OSError as exc:
            self.get_logger().error(
                f"cannot reach the broker at {host}:{port} ({exc}). "
                "Start one:  mosquitto -p 1883 -v")
            return False
        self.client.loop_start()

        period = float(self.get_parameter("status_period").value)
        self.create_timer(period, self._beat)
        return True

    def _beat(self) -> None:
        if self.link is None:
            return
        try:
            self.link.status(True, "busy" if self._busy.locked() else "idle")
        except DriveError:
            pass                    # no odometry yet; the next beat will do

    # ------------------------------------------------------------ missions
    def _take(self, payload: bytes) -> None:
        """A request arrived on the MQTT thread. Verify here, drive there."""
        if self._busy.locked():
            self.get_logger().warn("a mission is already running; "
                                   "the new request will be refused")
        mission = self.link.on_request(payload)
        if mission is not None:
            threading.Thread(target=self._run, args=(mission,),
                             daemon=True).start()

    def _run(self, mission: Mission) -> None:
        with self._busy:
            self.get_logger().info(
                f"{mission.request_id}: {mission.total} station(s)")
            t0 = time.time()
            try:
                self.driver.wait_ready()
                self.link.run_mission(self._minutes)
            except DriveError as exc:
                self.get_logger().error(f"mission aborted: {exc}")
                self.link.ack(mission.request_id, "failed", detail=str(exc))
            finally:
                self.driver.stop()
            self.get_logger().info(
                f"{mission.request_id}: done in {time.time() - t0:.0f} s, "
                f"floor camera used at {self.driver.visual_used} station(s), "
                f"unavailable at {self.driver.visual_missed}")

    def _minutes(self) -> float:
        """Minutes since midnight, on SIM time -- the sensor field's diurnal
        cycle should follow the simulation, not the wall clock."""
        now = self.get_clock().now().nanoseconds * 1e-9
        return (now % 86400.0) / 60.0

    def run_standalone(self) -> None:
        """Drive a list of stations with no Cloud. Diagnostic only."""
        self.get_logger().warn(
            "standalone_targets is set: driving without a Cloud. Reports are "
            "built and DISCARDED -- nothing is transmitted.")
        self.driver.wait_ready()
        for label in self.standalone:
            try:
                _envelope, note = self.visitor.visit(label.strip(),
                                                     self._minutes())
                s = self.driver.last_sighting
                seen = (f", floor camera saw the cross at "
                        f"{s.range * 1000:.0f} mm" if s else
                        ", no cross in the floor camera")
                self.get_logger().info(note + seen)
            except Exception as exc:                 # noqa: BLE001
                self.get_logger().error(f"{label}: {exc}")
        self.driver.stop()
        self.get_logger().info("standalone run finished")

    def shutdown(self) -> None:
        try:
            self.driver.stop()
            if self.link is not None:
                self.link.status(False, "shutting down")
        finally:
            if self.client is not None:
                self.client.loop_stop()
                self.client.disconnect()


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = AgriRobot()
    try:
        if node.standalone:
            threading.Thread(target=node.run_standalone, daemon=True).start()
        elif not node.connect():
            return 1
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
