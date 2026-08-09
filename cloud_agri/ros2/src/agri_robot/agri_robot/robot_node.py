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

from agri import keys, log, trust
from agri.protocol import (DEFAULT_BROKER, QOS, TOPIC_REQUEST,
                           BrokerAuth, ProtocolError, mqtt_client,
                           topic_query, topic_status)
from agri.replay import FRESHNESS_S
from agri.robot import Mission, RobotLink, Visitor
from agri.sensors import GreenhouseField
from agri_robot.driver import DriveError, GazeboDriver

#: How long to wait for the broker before saying so, and how often to repeat
#: it. Long enough that the two scripts starting together do not produce a
#: complaint every time; short enough that a broker nobody started is named
#: while the operator is still watching the window.
BROKER_GRACE_S = 20.0


class AgriRobot(Node):
    def __init__(self) -> None:
        super().__init__("agri_robot")
        self.declare_parameter("robot_id", "youbot-01")
        self.declare_parameter("broker", DEFAULT_BROKER)
        # 0 means "decide from the transport": 8883 with TLS, 1883 without.
        # A ROS parameter cannot be None, so 0 is the sentinel.
        self.declare_parameter("broker_port", 0)
        # TLS and broker credentials. Off by default -- the demonstration
        # runs mosquitto on the same laptop -- but a flag away, so the step
        # to a deployment is configuration and not a rewrite. The password
        # is read from $AGRI_BROKER_PASSWORD and never from a parameter,
        # because a ROS parameter is readable by `ros2 param get` from any
        # node on the graph.
        self.declare_parameter("broker_ca", "")
        self.declare_parameter("broker_cert", "")
        self.declare_parameter("broker_key", "")
        self.declare_parameter("broker_user", "")
        self.declare_parameter("broker_insecure", False)
        self.declare_parameter("keys", "keys")
        self.declare_parameter("status_period", 5.0)
        # How old a signed order may be before the robot refuses it as a
        # replay. Raise it only if the two machines' clocks genuinely
        # cannot be synced -- the refusal says by how much they disagree.
        self.declare_parameter("replay_window", FRESHNESS_S)
        # Drive a list of stations with no Cloud at all. For bringing the
        # simulator up and watching the robot find its crosses before any
        # broker exists -- the reports are built and thrown away.
        self.declare_parameter("standalone_targets", "")
        # A LIVE, real sensor failure rate -- 0.0 (off) unless a
        # demonstration asks for it. This is not the dashboard's "simulate a
        # failure" button, which only changes what is DISPLAYED for a
        # station that measured fine; this makes the read itself genuinely
        # fail sometimes, the same way a dead sensor board would, so the
        # Cloud's automatic LSTM fallback (agri.cloud.server.Cloud.on_ack)
        # can be demonstrated against a real gap in the data.
        self.declare_parameter("fail_rate", 0.0)

        def p(name):
            return self.get_parameter(name).value

        self.robot_id = p("robot_id")
        key_dir = Path(p("keys"))
        # See keys.bootstrap. generate=False on the CLOUD's public key is
        # what stops a robot running on its own machine from inventing a
        # Cloud identity and then refusing every real order as unsigned --
        # which looks exactly like a broker that is not delivering.
        keys.bootstrap(key_dir)
        self.robot_keys = keys.ensure("robot", key_dir, generate=False)
        self.cloud_public = keys.public_of("cloud", key_dir, generate=False)
        # A revoked key still verifies -- that is why it has to be listed
        # rather than deleted. Refusing to start is deliberate: a robot
        # that logged a warning and then took orders from a retired Cloud
        # key would make the revocation list decorative.
        store = trust.TrustStore(key_dir)
        store.check(self.cloud_public, "cloud")
        store.check(self.robot_keys.public_pem, "robot (our own)")
        for role, pem in (("robot", self.robot_keys.public_pem),
                          ("cloud", self.cloud_public)):
            self.get_logger().info(trust.summarise(role, pem, key_dir))
            warn = trust.age_warning(role, key_dir)
            if warn:
                self.get_logger().warning(warn)

        field = GreenhouseField(fail_rate=float(p("fail_rate")))
        if field.fail_rate:
            self.get_logger().warning(
                f"{log.warn('live fault injection is ON')}: "
                f"{field.fail_rate:.0%} of visits will genuinely fail")
        self.driver = GazeboDriver(self, sensors=field.read,
                                   log=self.get_logger().info)
        self.visitor = Visitor(robot_id=self.robot_id,
                               cloud_public_pem=self.cloud_public,
                               robot_private_pem=self.robot_keys.private_pem,
                               read_sensors=field.read,
                               driver=self.driver)

        self.get_logger().info(
            f"{log.bold('agri_robot')} {log.data(self.robot_id)}: "
            f"keys in {log.dim(str(key_dir.resolve()))}, "
            f"readings are {log.warn('SYNTHESISED')} by agri.sensors")

        self.client = None
        self.link: RobotLink | None = None
        self._busy = threading.Lock()
        self._connected = False
        #: Set at shutdown, so the broker watchdog does not complain about a
        #: node that is on its way out.
        self._quiet = threading.Event()
        self.standalone = [t for t in str(p("standalone_targets")).split(";")
                           if t.strip()]

    # ------------------------------------------------------------- startup
    def connect(self) -> None:
        """Attach to the broker, and KEEP TRYING until it is there.

        This used to give up: one failed connect() and main() returned 1, so
        the node died about a second after the simulator opened. The rest of
        the launch stayed up, Gazebo rendered a greenhouse, RViz drew the
        robot, and every request issued from the Cloud went nowhere -- with
        the only evidence eleven lines up in a log that had since scrolled
        past. "My requests are not being executed" is what that looks like
        from the outside, and nothing on screen said why.

        Starting the broker first fixes the common case and is what the
        launch scripts now do. It does not fix the case worth designing for:
        the broker restarting, or coming up a moment late, at any point in a
        forty-minute run. paho already reconnects after the FIRST successful
        connection; connect_async + loop_start extends that to the first one
        as well, so the ordering of two terminals stops being load-bearing.
        """
        host = self.get_parameter("broker").value
        # DELIBERATELY "agri-{robot_id}", not per-process. Giving every
        # process its own id (a pid suffix, say) would let a leftover
        # robot_node from an earlier run sit on the broker forever,
        # invisibly, alongside a fresh one -- both acting on the SAME
        # broadcast requests and driving the SAME simulated robot's wheels
        # at once, with a perfectly quiet log. What actually happened once
        # was noisy instead: the broker kicked whichever client reconnected
        # last, over and over ("connected" / "the broker went away" every
        # second), and that noise is what surfaced two processes fighting
        # over one identity before it was diagnosed as anything worse than
        # "the network is flaky". run_sim.sh now refuses to start a second
        # robot_node in the first place; this identity clash is the reason
        # that guard exists, and the fixed id is what makes a bypass loud
        # rather than a silent double-drive.
        self.client = mqtt_client(f"agri-{self.robot_id}")

        auth = BrokerAuth.from_env(
            ca=str(self.get_parameter("broker_ca").value),
            cert=str(self.get_parameter("broker_cert").value),
            key=str(self.get_parameter("broker_key").value),
            username=str(self.get_parameter("broker_user").value),
            insecure=bool(self.get_parameter("broker_insecure").value))
        try:
            self.get_logger().info(auth.apply(self.client))
        except ProtocolError as exc:
            # Fatal on purpose. A node that fell back to plaintext after
            # being told to use TLS would be the worst of both: an operator
            # who believes the link is protected and a link that is not.
            raise SystemExit(f"agri_robot: {exc}") from exc
        given = int(self.get_parameter("broker_port").value)
        port = auth.port(given or None)
        self.link = RobotLink(
            self.visitor, self.cloud_public, self.client,
            log=self.get_logger().info,
            replay_window_s=float(self.get_parameter("replay_window").value))

        # Last will: if this process dies, the broker tells the Cloud. A
        # dashboard that shows a robot as online because nobody said
        # otherwise is worse than one that shows nothing.
        self.client.will_set(topic_status(self.robot_id), json.dumps({
            "schema": "agri-cloud/v1", "kind": "status",
            "node_kind": "mobile", "robot": self.robot_id, "online": False,
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
            # Re-subscribe on EVERY connect, not just the first. A reconnect
            # to a broker that restarted starts from an empty subscription
            # table, and a robot that is connected and subscribed to nothing
            # is indistinguishable from one that is working.
            cl.subscribe(TOPIC_REQUEST, qos=QOS)
            # The node's OWN query topic, not a wildcard: a query names
            # one node, and a second robot must not answer this one's.
            cl.subscribe(topic_query(self.robot_id), qos=QOS)
            first, self._connected = not self._connected, True
            tag = log.ok("connected") if first else log.warn("RECONNECTED")
            self.get_logger().info(
                f"{tag} to {log.data(host)}:{log.data(str(port))}, "
                f"listening on {log.dim(TOPIC_REQUEST)}")
            self.link.status(True, "ready")

        @guarded
        def on_disconnect(_cl, _u, rc, *_):
            self._connected = False
            self.get_logger().warn(
                f"{log.err('lost the broker')} at {host}:{port} (rc={rc}); "
                f"{log.warn('retrying')}. Requests issued meanwhile will not arrive.")

        @guarded
        def on_message(_cl, _u, msg):
            if msg.topic == TOPIC_REQUEST:
                self._take(msg.payload)
            elif msg.topic == topic_query(self.robot_id):
                self.link.on_query(msg.payload)

        self.client.on_connect, self.client.on_message = on_connect, on_message
        self.client.on_disconnect = on_disconnect
        self.client.reconnect_delay_set(min_delay=1, max_delay=15)
        # connect_async, so a broker that is not up yet is a wait rather than
        # a fatal error: loop_start retries the first connection too.
        self.client.connect_async(host, port, keepalive=30)
        self.client.loop_start()
        self.get_logger().info(
            f"reaching for the broker at {log.data(host)}:{log.data(str(port))}")

        # A WALL-CLOCK timer. use_sim_time is true for this node, so a ROS
        # timer only advances when /clock does -- and the state this watchdog
        # reports is "nothing has connected", which is exactly the state a
        # half-started system is in. Same reasoning as viz_node's.
        def nag() -> None:
            if self._quiet.is_set() or self._connected:
                return
            self.get_logger().error(
                f"still no broker at {host}:{port} after "
                f"{BROKER_GRACE_S:.0f} s. The robot is up and will act on "
                "requests the moment one appears, but nothing can reach it "
                "until then. Start one:  mosquitto -p 1883 -v")
            arm(nag)

        def arm(fn) -> None:
            t = threading.Timer(BROKER_GRACE_S, fn)
            t.daemon = True
            t.start()

        arm(nag)

        period = float(self.get_parameter("status_period").value)
        self.create_timer(period, self._beat)

    def _beat(self) -> None:
        if self.link is None or not self._connected:
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
                f"{log.bold('mission')} {log.dim(mission.request_id)}: "
                f"{log.data(str(mission.total))} station(s)")
            t0 = time.time()
            try:
                self.driver.wait_ready()
                self.link.run_mission(self._minutes)
            except DriveError as exc:
                self.get_logger().error(f"mission aborted: {exc}")
                self.link.ack(mission.request_id, "failed", detail=str(exc))
            finally:
                self.driver.stop()
            elapsed = time.time() - t0
            msg = (f"{log.ok('done')} {log.dim(mission.request_id)} "
                   f"in {log.data(f'{elapsed:.0f}')} s, "
                   f"camera used at {log.data(str(self.driver.visual_used))}, "
                   f"unavailable at {self.driver.visual_missed}")
            if self.driver.visual_unsettled:
                msg += (f", {log.warn('NOT SETTLED')} at "
                        f"{self.driver.visual_unsettled}")
            self.get_logger().info(msg)

    def _minutes(self) -> float:
        """Minutes since midnight, on SIM time -- the sensor field's diurnal
        cycle should follow the simulation, not the wall clock."""
        now = self.get_clock().now().nanoseconds * 1e-9
        return (now % 86400.0) / 60.0

    def run_standalone(self) -> None:
        """Drive a list of stations with no Cloud. Diagnostic only."""
        self.get_logger().warn(
            f"{log.warn('standalone mode')}: driving without a Cloud. "
            f"Reports are built and {log.warn('DISCARDED')}.")
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
        self._quiet.set()
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
        else:
            node.connect()
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
