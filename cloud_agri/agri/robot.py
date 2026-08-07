"""robot -- the mobile node: takes requests, visits crosses, sends reports.

TWO BODIES, ONE BRAIN

The same robot logic has to run in two places: inside a ROS 2 node driving a
youbot in Gazebo, and inside a plain Python process for demonstrating the
Cloud without a simulator. Writing it twice guarantees they diverge, and the
one that gets debugged is whichever was run last.

So everything that is not motion lives here, transport- and ROS-free:

    Visitor     what to do at a station: read, photograph, encode, seal
    Mission     what to do with a request: the queue, the acks, the retries
    RobotLink   the MQTT wiring, given callbacks for "drive there" and
                "take a picture"

A ROS 2 node supplies a drive callback that publishes a goal and waits; the
offline simulator supplies one that teleports after a plausible delay. Both
produce byte-identical reports, which is what makes the offline demo
evidence about the real system rather than a mock-up of it.

WHAT THE ROBOT REFUSES

An unsigned request, or one signed by a key that is not the Cloud's. Without
that check anybody able to reach the broker could drive the robot. It is one
line, and leaving it out is the difference between a network node and an
open relay.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from typing import Callable, Protocol

from agri.aisles import HEADLAND_WEST
from agri.catalogue import Station, nearest_station, station
from agri.crypto_ecc import CryptoError, verify_json
from agri.envelope import PARK_TOLERANCE, build_report, seal_report, synthetic_photo
from agri.measurement import Measurement
from agri.protocol import (QOS, TOPIC_ACK, TOPIC_REPORT, TOPIC_REQUEST,
                           TOPIC_STATUS, ProtocolError, check_request,
                           make_ack, make_status)

Pose = tuple[float, float, float]          # x, y, yaw(rad)


class Driver(Protocol):
    """Whatever actually moves the base."""

    def drive_to(self, s: Station) -> Pose:
        """Go to the cross and return where the robot ENDED UP.

        Returning the achieved pose rather than the requested one is the
        whole contract: the report carries where the robot really was, the
        Cloud compares it with the catalogue, and a robot that stopped 40 cm
        short is visible instead of being taken at its word.
        """

    def photograph(self, s: Station) -> tuple[bytes, tuple[int, int]]:
        """(jpeg bytes, (width, height)) of the plant at this station."""

    def pose(self) -> Pose:
        ...

    def velocity(self) -> tuple[float, float, float]:
        """(vx, vy, wz) in the robot's own frame, right now.

        Reported with every measurement because the brief asks the robot to
        send its position AND its speed, and because a reading taken while
        moving is a reading taken somewhere between two places. It should be
        near zero at a station -- that is the point of transmitting it.
        """


@dataclass
class Visitor:
    """One station, start to sealed envelope. No transport, no motion."""

    robot_id: str
    cloud_public_pem: bytes
    robot_private_pem: bytes
    read_sensors: Callable[[str, float, Pose], Measurement]
    driver: Driver
    #: How far off the cross is still acceptable. Two stations in an inner
    #: aisle are 0.10 m apart, so this must stay well under half of that or
    #: a visit gets filed under its neighbour's label.
    park_tolerance: float = PARK_TOLERANCE

    def visit(self, label: str, minutes: float,
              request_id: str | None = None) -> tuple[dict, str]:
        """(sealed envelope, one-line note). Raises nothing the caller must
        handle: a bad park is reported, not hidden, and the Cloud decides."""
        s = station(label)
        pose = self.driver.drive_to(s)
        err = math.hypot(pose[0] - s.x, pose[1] - s.y)

        m = self.read_sensors(label, minutes, pose)
        # Read the speed AFTER the sensors, not before: it is the speed at
        # the moment of the reading that says whether the reading is a point
        # measurement or a smear.
        m.velocity = self.driver.velocity()
        jpeg, size = self.driver.photograph(s)
        report = build_report(m, jpeg, size, self.robot_id, request_id)
        envelope = seal_report(report, self.cloud_public_pem,
                               self.robot_private_pem)

        note = f"{label} parked {err:.3f} m from the cross"
        if err > self.park_tolerance:
            near, d = nearest_station(pose[0], pose[1])
            note += (f" -- OUTSIDE tolerance {self.park_tolerance:.2f} m"
                     + (f", and {near.label} is nearer ({d:.3f} m)"
                        if near.label != label else ""))
        return envelope, note


@dataclass
class Mission:
    """The queue behind a request. Deliberately dumb: one station at a time,
    in the order the Cloud sent them, no reordering.

    Reordering to shorten the route is the obvious improvement and it is
    left out on purpose here: the Cloud already emits the stations in survey
    order, and a robot that silently reorders makes the ack stream stop
    matching the request, which is the first thing an operator reads.
    """

    request_id: str
    targets: list[str]
    done: int = 0
    failures: list[str] = field(default_factory=list)

    @property
    def total(self) -> int:
        return len(self.targets)

    @property
    def finished(self) -> bool:
        return self.done >= self.total


class RobotLink:
    """MQTT wiring. Given a Visitor, it is a complete robot node."""

    def __init__(self, visitor: Visitor, cloud_public_pem: bytes,
                 client, log: Callable[[str], None] = print) -> None:
        self.visitor = visitor
        self.cloud_public_pem = cloud_public_pem
        self.client = client
        self.log = log
        self.mission: Mission | None = None
        self.started = time.time()

    # ------------------------------------------------------------ inbound
    def on_request(self, payload: bytes) -> Mission | None:
        """Verify and accept a request. Returns the Mission, or None."""
        try:
            signed = json.loads(payload)
        except Exception as exc:                     # noqa: BLE001
            self.log(f"robot: request is not JSON ({exc})")
            return None
        try:
            body = verify_json(signed, self.cloud_public_pem)
        except CryptoError as exc:
            # The important refusal. Anyone can publish to the topic; only
            # the Cloud can sign.
            self.log(f"robot: REFUSED an unsigned or forged request ({exc})")
            return None
        try:
            rid, targets = check_request(body)
        except ProtocolError as exc:
            self.log(f"robot: malformed request ({exc})")
            return None

        if self.mission and not self.mission.finished:
            self.log(f"robot: busy with {self.mission.request_id}, "
                     f"queuing {rid} is not supported -- refusing")
            self.ack(rid, "failed", detail="robot busy")
            return None

        self.mission = Mission(request_id=rid, targets=targets)
        self.log(f"robot: accepted {rid}, {len(targets)} station(s)")
        self.ack(rid, "accepted", total=len(targets))
        return self.mission

    # ----------------------------------------------------------- outbound
    def ack(self, request_id: str, state: str, label: str | None = None,
            total: int = 0, detail: str = "") -> None:
        m = self.mission
        self.client.publish(TOPIC_ACK, json.dumps(make_ack(
            request_id, self.visitor.robot_id, state, label,
            done=m.done if m else 0,
            total=total or (m.total if m else 0), detail=detail)), qos=QOS)

    def status(self, online: bool = True, note: str = "") -> None:
        """Telemetry. Pose and velocity, on a timer, whether busy or not --
        so the Cloud can watch the robot move rather than only hear from it
        when it arrives somewhere.

        NEVER RAISES. This is called from the MQTT client's own callback the
        moment the connection is established, which is before the simulator
        has published a single odometry message. The first version let the
        driver's "no odometry yet" propagate, and paho -- which does not
        guard its callbacks -- killed its network thread on the way out. The
        robot then sat there looking healthy: its timer kept publishing
        status, because that runs on a different thread, while every request
        the Cloud sent was delivered by the broker to a client that was no
        longer reading. Forty minutes of a robot that will not move and a
        log that says nothing.

        A status with no position is still worth sending: "I am here and I
        am connected" is exactly what the Cloud needs at that instant.
        """
        pose = velocity = None
        try:
            p = self.visitor.driver.pose()
            pose = (p[0], p[1], math.degrees(p[2]))
            velocity = self.visitor.driver.velocity()
        except Exception as exc:                     # noqa: BLE001
            note = f"{note} (no odometry yet: {exc})".strip()
        self.client.publish(TOPIC_STATUS, json.dumps(make_status(
            self.visitor.robot_id, online, pose, note, velocity=velocity)),
            qos=QOS, retain=True)

    def run_mission(self, minutes_provider: Callable[[], float]) -> None:
        """Work the queue to the end, publishing as it goes."""
        m = self.mission
        if m is None:
            return
        for label in m.targets:
            self.ack(m.request_id, "driving", label)
            try:
                envelope, note = self.visitor.visit(
                    label, minutes_provider(), m.request_id)
            except Exception as exc:                 # noqa: BLE001
                m.failures.append(label)
                m.done += 1
                self.log(f"robot: {label} FAILED ({exc})")
                self.ack(m.request_id, "failed", label, detail=str(exc))
                continue
            self.client.publish(TOPIC_REPORT, json.dumps(envelope), qos=QOS)
            m.done += 1
            self.log(f"robot: {note}")
            self.ack(m.request_id, "sent", label)
        state = "finished" if not m.failures else "finished_with_failures"
        self.ack(m.request_id, state,
                 detail=f"{len(m.failures)} failure(s)" if m.failures else "")
        self.log(f"robot: {m.request_id} {state} "
                 f"({m.done}/{m.total})")


# ----------------------------------------------------------- offline body
class SimulatedDriver:
    """A body with no simulator: it goes where it is told, imperfectly.

    The parking error is the point. A robot that always lands exactly on the
    cross would make every downstream check pass vacuously -- the Cloud's
    'did you park on the station you claim?' test would never once fire, and
    nobody would know whether it worked. So this one lands with a small,
    seeded error, and the tolerance is genuinely exercised.
    """

    def __init__(self, seed: int = 7, park_sigma: float = 0.012,
                 speed: float = 0.45, dwell: float = 0.0) -> None:
        import random                                # noqa: PLC0415
        self.rng = random.Random(seed)
        self.park_sigma = park_sigma
        self.speed = speed
        self.dwell = dwell
        self._pose: Pose = (HEADLAND_WEST, -1.85, 0.0)   # the dock, west end
        self._velocity = (0.0, 0.0, 0.0)

    def pose(self) -> Pose:
        return self._pose

    def drive_to(self, s: Station) -> Pose:
        if self.dwell:
            travel = math.hypot(s.x - self._pose[0], s.y - self._pose[1])
            time.sleep(min(self.dwell, travel / max(self.speed, 1e-3)))
        yaw = 0.0 if s.x >= self._pose[0] else math.pi
        self._pose = (s.x + self.rng.gauss(0, self.park_sigma),
                      s.y + self.rng.gauss(0, self.park_sigma), yaw)
        self._velocity = (self.rng.gauss(0, 0.004),
                          self.rng.gauss(0, 0.004), 0.0)
        return self._pose

    def velocity(self) -> tuple[float, float, float]:
        """Parked, with the residual wobble of a base that has just stopped.

        Not exactly zero, on purpose: a field that is always the same number
        is a field nobody looks at, and one that is always EXACTLY zero would
        hide the day the robot starts measuring while still rolling.
        """
        return self._velocity

    def photograph(self, s: Station) -> tuple[bytes, tuple[int, int]]:
        return synthetic_photo(s.label), (320, 240)
