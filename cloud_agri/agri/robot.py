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
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Protocol

from agri.aisles import HEADLAND_WEST
from agri.catalogue import Station, nearest_station, station
from agri.crypto_ecc import CryptoError, verify_json
from agri.envelope import PARK_TOLERANCE, build_report, seal_report, synthetic_photo
from agri.measurement import Measurement
from agri.protocol import (MODE_COLLECTOR, MODE_COMMAND, QOS, STEP_FILED,
                           STEP_HOLD, STEP_IDLE, STEP_IDLE_ASK, STEP_OFFER,
                           STEP_SEND, TOPIC_REQUEST, ProtocolError,
                           check_request, make_ack, make_reply, make_status,
                           request_mode, topic_ack, topic_reply, topic_report,
                           topic_status)

from agri.survey import crossings, path_length, survey_order

#: How many sealed reports the robot will hold while the Cloud is not
#: taking them. Forty-eight is one full campaign: the robot can measure the
#: whole greenhouse with the Cloud down and hand it all over afterwards.
#: Bounded rather than unbounded because a queue with no limit turns a
#: Cloud outage into a robot that runs out of memory mid-aisle, and the
#: oldest reading is the one worth dropping first.
OUTBOX_MAX = 48

#: How long the robot waits for permission to send before deciding the
#: Cloud is not listening and keeping the reading. Short: the Cloud answers
#: in milliseconds when it is there at all, and a robot standing at a mark
#: waiting on a dead Cloud is worse than one that carries on measuring.
GRANT_TIMEOUT_S = 5.0

#: Speed below which the robot calls itself stopped. Odometry noise on this
#: platform sits around a millimetre per second; 0.01 m/s is clear of it and
#: far below anything that would smear a reading.
STILL_MPS = 0.01

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
    """The queue behind a request: one station at a time, to the end.

    THE ORDER IS THE ROBOT'S TO CHOOSE, AND IT SAYS SO OUT LOUD.

    This used to drive the stations exactly as the Cloud listed them, and
    carried a note saying reordering was left out on purpose so that the ack
    stream would keep matching the request. That reasoning was worth about
    228 metres a sweep.

    Survey order -- P1,1R, P1,1L, P1,2R, P1,2L -- is right for a human
    reading a list and wrong for a robot driving a building. The two sides of
    one plant are separated by an 8 m gutter, so R to L means going to the
    end of the row, crossing, and coming back; the survey order asks for that
    forty-five times, and 87 % of the distance in a 48-station campaign was
    that and nothing else.

    The objection was real, so it is answered rather than accepted: `issued`
    keeps the order as the Cloud sent it, every ack still names its station,
    and the Cloud reports progress by label. Nothing an operator reads was
    ever indexed by position.
    """

    request_id: str
    targets: list[str]
    #: command or collector. Comes out of the SIGNED request, so it is an
    #: instruction from the Cloud rather than a local setting.
    mode: str = MODE_COMMAND
    #: The order as received, kept so the two can be compared and so an
    #: operator can always be shown what they actually asked for.
    issued: list[str] = field(default_factory=list)
    #: Metres the chosen order is expected to cost, from where the robot
    #: stood when it accepted. Planned, not driven -- the driven figure is
    #: measured from odometry and reported beside it.
    planned_m: float = 0.0
    done: int = 0
    failures: list[str] = field(default_factory=list)

    def __post_init__(self) -> None:
        if not self.issued:
            self.issued = list(self.targets)

    @property
    def reordered(self) -> bool:
        return self.targets != self.issued

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
        #: Sealed reports the Cloud has not yet agreed to receive. The whole
        #: point of collector mode: the robot keeps working while the far
        #: side is busy, and hands everything over when it is asked to.
        #: Bounded, because a queue that grows without limit turns a Cloud
        #: outage into a robot that runs out of memory in the greenhouse.
        self.outbox: deque[tuple[str, dict]] = deque(maxlen=OUTBOX_MAX)
        self.dropped = 0
        self._grant = threading.Event()
        self._granted = False

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

        mode = request_mode(body)
        self.mission = Mission(request_id=rid, targets=targets, mode=mode)
        self.log(f"robot: accepted {rid}, {len(targets)} station(s) "
                 f"in {mode} mode")
        self.ack(rid, "accepted", total=len(targets))
        return self.mission

    # ----------------------------------------------------------- outbound
    def ack(self, request_id: str, state: str, label: str | None = None,
            total: int = 0, detail: str = "", metres: float | None = None,
            planned_m: float | None = None) -> None:
        m = self.mission
        self.client.publish(
            topic_ack(self.visitor.robot_id),
            json.dumps(make_ack(
                request_id, self.visitor.robot_id, state, label,
                done=m.done if m else 0,
                total=total or (m.total if m else 0), detail=detail,
                metres=metres, planned_m=planned_m)),
            qos=QOS)

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
        self.client.publish(
            topic_status(self.visitor.robot_id),
            json.dumps(make_status(self.visitor.robot_id, online, pose, note,
                                   velocity=velocity)),
            qos=QOS, retain=True)

    # -------------------------------------------------------- handshake
    def is_still(self) -> bool:
        """Is the base actually stopped, from its own odometry?

        Asked by the Cloud before it issues a collector order, so a mission
        never starts while the robot is still rolling from the last one.
        Never raises: no odometry at all answers as 'not stopped', which is
        the safe reading -- a robot that cannot say where it is should not
        be given somewhere to go.
        """
        try:
            vx, vy, _ = self.visitor.driver.velocity()
            return math.hypot(vx, vy) < STILL_MPS
        except Exception:                            # noqa: BLE001
            return False

    def reply(self, step: str, **extra) -> None:
        nid = self.visitor.robot_id
        self.client.publish(topic_reply(nid),
                            json.dumps(make_reply(step, nid, **extra)),
                            qos=QOS)

    def on_query(self, payload: bytes) -> None:
        """The Cloud asking, or granting.

        NEVER RAISES. This runs on paho's own callback thread, which paho
        does not guard: an exception here kills the network thread and
        leaves a robot that looks healthy and answers nothing. That exact
        failure cost forty minutes once already -- see status().
        """
        try:
            d = json.loads(payload)
            step = d.get("step")
            if step == STEP_IDLE_ASK:
                still = self.is_still()
                self.reply(STEP_IDLE, idle=still,
                           busy=bool(self.mission
                                     and not self.mission.finished),
                           holding=len(self.outbox))
                self.log("robot: Cloud asks if I am stopped -> "
                         + ("stopped" if still else "still moving"))
            elif step in (STEP_SEND, STEP_HOLD):
                self._granted = step == STEP_SEND
                self._grant.set()
            elif step == STEP_FILED:
                self.log(f"robot: Cloud filed {d.get('label', '?')}")
        except Exception as exc:                     # noqa: BLE001
            self.log(f"robot: bad query ({exc})")

    def _deliver(self, label: str, envelope: dict, mode: str) -> str:
        """Hand one sealed report over, or keep it until the Cloud asks.

        In COMMAND mode it goes straight out: the Cloud asked for it and is
        waiting. In COLLECTOR mode the robot offers it first and waits for
        permission, because the Cloud is driving the campaign and is
        entitled to say 'not now' -- and a robot that dumped readings at a
        Cloud that said hold would make the negotiation decorative.
        """
        nid = self.visitor.robot_id
        if mode != MODE_COLLECTOR:
            self.client.publish(topic_report(nid), json.dumps(envelope),
                                qos=QOS)
            return f"{label} sent"

        self._grant.clear()
        self._granted = False
        self.reply(STEP_OFFER, label=label, holding=len(self.outbox) + 1,
                   request_id=self.mission.request_id if self.mission else None)
        self.log(f"robot: parked and still at {label}, reading ready -- "
                 "asking the Cloud to receive")
        if not self._grant.wait(GRANT_TIMEOUT_S) or not self._granted:
            if len(self.outbox) == self.outbox.maxlen:
                self.dropped += 1
            self.outbox.append((label, envelope))
            why = "said hold" if self._grant.is_set() else "did not answer"
            self.log(f"robot: the Cloud {why} -- keeping {label}, "
                     f"{len(self.outbox)} reading(s) held")
            return f"{label} held"

        # Granted. Flush the held ones first, oldest first, so a Cloud that
        # comes back after an outage receives the campaign in the order it
        # actually happened rather than newest-first.
        held = len(self.outbox)
        while self.outbox:
            _, env = self.outbox.popleft()
            self.client.publish(topic_report(nid), json.dumps(env), qos=QOS)
        self.client.publish(topic_report(nid), json.dumps(envelope), qos=QOS)
        return (f"{label} sent" if not held
                else f"{label} sent, with {held} held reading(s)")

    def _odometer(self) -> float | None:
        """Metres the base has driven since it started, if it counts them.

        getattr rather than a method on the Driver protocol: the offline demo
        and the test suite supply their own drivers, and a new REQUIRED
        method would break every one of them for a number that is only ever
        reported. A driver that cannot say returns None and the mission says
        'planned' instead of 'driven'.
        """
        try:
            return float(self.visitor.driver.distance_m())
        except Exception:                            # noqa: BLE001
            return None

    def plan(self) -> None:
        """Choose the order to drive the accepted stations in.

        Separate from run_mission so the decision can be tested without a
        simulator, and taken HERE rather than when the request arrives
        because the best order depends on where the robot is standing now.
        """
        m = self.mission
        if m is None:
            return
        try:
            x, y, _ = self.visitor.driver.pose()
        except Exception as exc:                     # noqa: BLE001
            # No odometry yet is not a reason to refuse to drive. The issued
            # order is a valid order; it is only a longer one.
            self.log(f"robot: keeping the issued order ({exc})")
            return
        try:
            m.targets = survey_order(m.issued, x, y)
            m.planned_m = path_length(m.targets, x, y)
        except Exception as exc:                     # noqa: BLE001
            self.log(f"robot: could not reorder, driving as issued ({exc})")
            m.targets = list(m.issued)
            return
        if m.reordered:
            was = path_length(m.issued, x, y)
            self.log(f"robot: {m.request_id} reordered to save "
                     f"{was - m.planned_m:.0f} m "
                     f"({was:.0f} -> {m.planned_m:.0f} m, "
                     f"{crossings(m.issued, x, y)} -> "
                     f"{crossings(m.targets, x, y)} gutter crossings)")

    def run_mission(self, minutes_provider: Callable[[], float]) -> None:
        """Work the queue to the end, publishing as it goes."""
        m = self.mission
        if m is None:
            return
        self.plan()
        started_m = self._odometer()
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
            outcome = self._deliver(label, envelope, m.mode)
            m.done += 1
            self.log(f"robot: {note}")
            self.ack(m.request_id,
                     "held" if outcome.endswith("held") else "sent", label,
                     detail=outcome)
        state = "finished" if not m.failures else "finished_with_failures"
        # The distance is MEASURED where the driver counts it, and only
        # planned where it does not. Reported on the closing ack so the
        # Cloud can put it in requests.csv without asking for it.
        ended_m = self._odometer()
        driven = (round(ended_m - started_m, 2)
                  if started_m is not None and ended_m is not None else None)
        bits = []
        if m.failures:
            bits.append(f"{len(m.failures)} failure(s)")
        if driven is not None:
            bits.append(f"drove {driven:.1f} m")
        elif m.planned_m:
            bits.append(f"planned {m.planned_m:.1f} m")
        self.ack(m.request_id, state, detail="; ".join(bits),
                 metres=driven, planned_m=round(m.planned_m, 2) or None)
        self.log(f"robot: {m.request_id} {state} "
                 f"({m.done}/{m.total})"
                 + (f", {driven:.1f} m driven" if driven is not None else ""))


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
