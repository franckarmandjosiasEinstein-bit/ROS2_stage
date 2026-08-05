#!/usr/bin/env python3
"""Pre-flight checks for cloud_agri. Run before a demonstration, not after.

    python3 tests/check_cloud.py

Same discipline as the Phase B suite: every check is here because something
actually went wrong, and each one says what it expected and what it found.
Nothing here needs a broker, a simulator or a network -- the robot and the
Cloud are wired to each other in-process through a loopback broker, so the
WHOLE chain (request -> drive -> read -> QR -> seal -> transmit -> verify ->
store) is exercised in about a second.

The end-to-end test is the one that matters. Unit tests on a crypto function
and a label parser prove those two work; only the loop proves the robot and
the Cloud agree about what a station is, and that is where the interesting
failures live.
"""

from __future__ import annotations

import json
import os
import re
import sys
import tempfile
import xml.dom.minidom
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

FAILURES: list[str] = []
CHECKS = 0


def check(name: str, ok: bool, detail: str = "") -> None:
    global CHECKS
    CHECKS += 1
    if ok:
        print(f"  ok    {name}")
    else:
        print(f"  FAIL  {name}")
        for line in str(detail).strip().splitlines():
            print(f"          {line}")
        FAILURES.append(name)


def refuses(name: str, fn, kinds=Exception) -> None:
    """The check that matters most in a security-adjacent system: that the
    bad case is REFUSED. A test suite full of happy paths tells you the
    system works when nobody is attacking it."""
    global CHECKS
    CHECKS += 1
    try:
        fn()
    except kinds as exc:
        print(f"  ok    {name}")
        print(f"          refused: {str(exc).splitlines()[0][:78]}")
        return
    print(f"  FAIL  {name}")
    print("          it was ACCEPTED, which is a hole")
    FAILURES.append(name)


# =====================================================================
def check_labels() -> None:
    print("\nstation labels")
    from agri.labels import (LabelError, all_labels, format_label, normalise,
                             parse_label, station_count)

    check("48 stations: 3 rows x 8 plants x 2 sides", station_count() == 48)
    check("all_labels agrees", len(all_labels()) == 48)
    check("no duplicates", len(set(all_labels())) == 48)
    check("canonical form", format_label(2, 5, "R") == "P2,5R")
    check("a human's spelling is accepted",
          normalise("p2, 5 r") == "P2,5R" and normalise("P2-5L") == "P2,5L")
    check("round trip", all(normalise(l) == l for l in all_labels()))

    # Range checks: "P4,1R" parses cleanly and names a row that does not
    # exist. Accepting it would send the robot to a computed position in a
    # wall, so the parser refuses it rather than the caller remembering to.
    refuses("row 4 does not exist", lambda: normalise("P4,1R"), LabelError)
    refuses("plant 9 does not exist", lambda: normalise("P1,9R"), LabelError)
    refuses("side M does not exist", lambda: normalise("P1,1M"), LabelError)
    refuses("empty is not a label", lambda: normalise(""), LabelError)


def check_catalogue() -> None:
    print("\ncatalogue geometry")
    import math

    from agri.catalogue import (BASE_HALF_WIDTH, GUTTER_HALF_WIDTH, ROW_Y,
                                all_stations, clearances, nearest_station,
                                plant_position, station, worst_clearance)

    st = all_stations()
    check("48 stations", len(st) == 48)
    check("every position distinct",
          len({(round(s.x, 4), round(s.y, 4)) for s in st}) == 48)

    # The catalogue claims to describe the Phase B greenhouse. Read that
    # world and check, so the two cannot drift apart unnoticed.
    world = ROOT.parent / "src/youbot_gazebo/worlds/greenhouse.sdf"
    if world.exists():
        sdf = world.read_text()
        found = set()
        for m in re.finditer(
                r'<model name="plant[^"]*">\s*<static>[^<]*</static>\s*'
                r'<pose>([-\d. ]+)</pose>', sdf):
            p = m.group(1).split()
            found.add((round(float(p[0]), 2), round(float(p[1]), 2)))
        ours = {(round(plant_position(i, j)[0], 2),
                 round(plant_position(i, j)[1], 2))
                for i in range(1, 4) for j in range(1, 9)}
        check("plant positions match the Phase B world", found == ours,
              f"world has {sorted(found - ours)[:3]}, "
              f"catalogue has {sorted(ours - found)[:3]}")
    else:
        check("Phase B world found for cross-checking", False, str(world))

    worst, margin = worst_clearance()
    check("every station clears the gutters and walls by >= 0.15 m",
          margin >= 0.15,
          f"{worst.label} has only {margin:.3f} m -- a wheel is on a gutter")

    # The base stands ALONG the aisle. If someone changes that back, the
    # half-length points across and the margin collapses; this is the check
    # that caught it the first time.
    tight = min(abs(abs(s.y - s.plant_y) - BASE_HALF_WIDTH - GUTTER_HALF_WIDTH)
                for s in st)
    check("clearance is computed from the half-WIDTH, not the half-length",
          abs(tight - margin) < 0.05 or margin >= 0.15,
          "turning the base to face the plant leaves ~1 cm; it must stay "
          "along the aisle and pan the head instead")

    # Pan angle must work in BOTH directions of travel, or half the visits
    # photograph the opposite row.
    s = station("P2,1R")
    check("head pans +90 deg driving east at an R station",
          abs(math.degrees(s.pan_for(0.0)) - 90) < 1e-6)
    check("head pans -90 deg driving west at the same station",
          abs(math.degrees(s.pan_for(math.pi)) + 90) < 1e-6)
    check("an L station looks the other way",
          abs(math.degrees(station('P2,1L').pan_for(0.0)) + 90) < 1e-6)

    # nearest_station is what catches a visit filed under the wrong label.
    for s in st:
        near, d = nearest_station(s.x, s.y)
        if near.label != s.label:
            check(f"nearest_station({s.label}) returns itself", False,
                  f"returned {near.label}")
            break
    else:
        check("nearest_station returns the station itself, for all 48", True)

    pairs = min(((a, b, math.hypot(a.x - b.x, a.y - b.y))
                 for i, a in enumerate(st) for b in st[i + 1:]),
                key=lambda t: t[2])
    check("the closest pair is far enough apart to tell apart",
          pairs[2] > 0.08,
          f"{pairs[0].label} and {pairs[1].label} are {pairs[2]:.3f} m apart")


def check_crypto() -> None:
    print("\nECC: confidentiality and origin")
    import base64

    from agri.crypto_ecc import (CryptoError, generate_keypair, seal_json,
                                 sign_json, unseal_json, verify_json)

    cloud, robot, intruder = (generate_keypair() for _ in range(3))
    msg = {"label": "P2,5R", "t": 21.4}
    env = seal_json(msg, cloud.public_pem, robot.private_pem)

    check("a sealed payload round-trips",
          unseal_json(env, cloud.private_pem, robot.public_pem) == msg)
    check("the plaintext is not in the envelope",
          "P2,5R" not in json.dumps(env))
    refuses("the wrong recipient key cannot open it",
            lambda: unseal_json(env, intruder.private_pem, robot.public_pem),
            CryptoError)
    refuses("a forged sender is rejected",
            lambda: unseal_json(env, cloud.private_pem, intruder.public_pem),
            CryptoError)
    refuses("stripping the signature does not bypass the check",
            lambda: unseal_json({k: v for k, v in env.items() if k != "sig"},
                                cloud.private_pem, robot.public_pem),
            CryptoError)

    flipped = dict(env)
    raw = bytearray(base64.b64decode(flipped["ciphertext"]))
    raw[4] ^= 0x01
    flipped["ciphertext"] = base64.b64encode(bytes(raw)).decode()
    refuses("one flipped bit is detected",
            lambda: unseal_json(flipped, cloud.private_pem, robot.public_pem),
            CryptoError)

    a = seal_json(msg, cloud.public_pem)
    b = seal_json(msg, cloud.public_pem)
    check("the same plaintext seals differently every time",
          a["ciphertext"] != b["ciphertext"] and a["epk"] != b["epk"],
          "a repeated ciphertext means the ephemeral key is not ephemeral")

    # Requests are signed and NOT encrypted: authenticity is the property
    # that matters there. Check the distinction is really implemented.
    signed = sign_json({"kind": "request", "targets": ["P1,1R"]},
                       cloud.private_pem)
    check("a signed request is readable by anyone",
          signed["payload"]["targets"] == ["P1,1R"])
    check("and verifies with the Cloud's key",
          verify_json(signed, cloud.public_pem)["kind"] == "request")
    refuses("but not with anyone else's",
            lambda: verify_json(signed, intruder.public_pem), CryptoError)
    tampered = {"payload": {"kind": "request", "targets": ["P3,8L"]},
                "sig": signed["sig"], "sig_alg": signed["sig_alg"]}
    refuses("and a rewritten target breaks it",
            lambda: verify_json(tampered, cloud.public_pem), CryptoError)


def check_qr() -> None:
    print("\nQR: the numbers survive the round trip")
    from agri.labels import all_labels
    from agri.measurement import Measurement
    from agri.qrcodec import QRError, decode_b64, encode_b64

    def m(label):
        return Measurement(label=label, timestamp="2026-08-04T18:22:31Z",
                           values={"temperature": 21.4, "humidity": 63.2,
                                   "luminosity": 12480, "co2": 431,
                                   "ph": 6.42})

    # Every station, through a real PNG. This is the check that caught
    # OpenCV failing to LOCATE two of the 48 codes -- a 4 % silent loss that
    # a single-station test would never have seen.
    bad = []
    for label in all_labels():
        try:
            if decode_b64(encode_b64(m(label).to_qr_text())) != m(label).to_qr_text():
                bad.append(label)
        except QRError as exc:
            bad.append(f"{label} ({exc})")
    check("all 48 station codes encode and decode again", not bad,
          f"failed: {bad}")

    text = m("P2,5R").to_qr_text()
    check("the payload is human-readable when scanned",
          text.startswith("AGRI1|P2,5R|") and "t=21.4" in text, text)
    back = Measurement.from_qr_text(text)
    check("and parses back to the same numbers", back.to_qr_text() == text)
    refuses("a payload from another system is rejected",
            lambda: Measurement.from_qr_text("HELLO|P2,5R"))
    refuses("an unknown quantity is rejected",
            lambda: Measurement.from_qr_text(text + "|zz=1"))


def check_sensors() -> None:
    print("\nsensor field: structure, not noise")
    from agri.labels import all_labels
    from agri.sensors import ANOMALIES, GreenhouseField

    f = GreenhouseField()
    check("every station reads", all(f.read(l) for l in all_labels()))
    check("the same station reads the same twice",
          f.read("P2,5R").values == f.read("P2,5R").values,
          "a re-read that differs means there is no underlying truth")
    check("neighbouring plants correlate",
          abs(f.truth("P2,4R")["temperature"] - f.truth("P2,5R")["temperature"])
          < 1.0,
          "random numbers would not")

    # The injected faults must actually stand out, or a demonstration that
    # 'found' them found noise.
    for label, (quantity, delta) in ANOMALIES.items():
        row, plant = int(label[1]), int(label.split(",")[1][:-1])
        others = [f.truth(f"P{row},{j}{label[-1]}")[quantity]
                  for j in range(1, 9) if j != plant]
        here = f.truth(label)[quantity]
        typical = sum(others) / len(others)
        check(f"the {quantity} anomaly at {label} stands out",
              abs(here - typical) > abs(delta) * 0.5,
              f"{here:.1f} against a neighbourhood of {typical:.1f}")


def check_world() -> None:
    print("\ngenerated world")
    from agri.catalogue import all_stations
    from agri.world.make_world import build, default_source

    source = default_source()
    if not source.exists():
        check("Phase B world available", False, str(source))
        return
    with tempfile.TemporaryDirectory() as d:
        target = Path(d) / "greenhouse_cloud.sdf"
        n, _ = build(source, target)
        check("48 crosses generated", n == 48)
        try:
            xml.dom.minidom.parse(str(target))
            check("the SDF is well-formed XML", True)
        except Exception as exc:                     # noqa: BLE001
            # '--' inside an XML comment is illegal and the error points at a
            # column in a generated file. Phase B lost two sessions to it and
            # the first draft of the generator walked into it again.
            check("the SDF is well-formed XML", False, str(exc))
        sdf = target.read_text()
        check("the plants survived", sdf.count('<model name="plant') == 24)
        for s in all_stations()[:6]:
            name = s.label.replace(",", "_")
            if f'<model name="cross_{name}">' not in sdf:
                check(f"cross_{name} is in the world", False)
                break
        else:
            check("the crosses are named after their stations", True)
        # The world's numbers must BE the catalogue's numbers.
        drift = []
        for s in all_stations():
            name = s.label.replace(",", "_")
            m = re.search(rf'<model name="cross_{name}">.*?<pose>'
                          r'([-\d.]+) ([-\d.]+)', sdf, re.S)
            if not m or abs(float(m.group(1)) - s.x) > 1e-3 \
                    or abs(float(m.group(2)) - s.y) > 1e-3:
                drift.append(s.label)
        check("every painted cross is where the catalogue says", not drift,
              f"{drift[:5]} differ -- the robot would stop beside them")
        refuses("regenerating from an already-generated world is refused",
                lambda: build(target, Path(d) / "again.sdf"), ValueError)


def check_end_to_end() -> None:
    """The one that matters: request -> drive -> seal -> verify -> store."""
    print("\nend to end, through a loopback broker")
    from agri import keys
    from agri.cloud.store import Store
    from agri.cloud.server import Cloud
    from agri.protocol import TOPIC_ACK, TOPIC_REPORT, TOPIC_REQUEST
    from agri.robot import RobotLink, SimulatedDriver, Visitor
    from agri.sensors import GreenhouseField

    with tempfile.TemporaryDirectory() as d:
        d = Path(d)
        keydir, storedir = d / "keys", d / "store"
        cloud_keys = keys.ensure("cloud", keydir)
        robot_keys = keys.ensure("robot", keydir)

        cloud = Cloud(Store(storedir), keydir)
        field = GreenhouseField()

        class Loopback:
            """A broker in a variable. Routes by topic, synchronously, so a
            failure has a stack trace instead of a timeout."""

            def __init__(self):
                self.robot = None

            def publish(self, topic, payload, qos=1, retain=False):
                if topic == TOPIC_REPORT:
                    self.last = cloud.on_report(payload.encode()
                                                if isinstance(payload, str)
                                                else payload)
                elif topic == TOPIC_ACK:
                    cloud.on_ack(payload.encode() if isinstance(payload, str)
                                 else payload)

        bus = Loopback()
        visitor = Visitor(robot_id="youbot-01",
                          cloud_public_pem=cloud_keys.public_pem,
                          robot_private_pem=robot_keys.private_pem,
                          read_sensors=field.read,
                          driver=SimulatedDriver(seed=3))
        link = RobotLink(visitor, cloud_keys.public_pem, bus, log=lambda s: None)

        # 1. the Cloud issues a signed request for six stations
        targets = ["P1,1R", "P1,1L", "P2,4R", "P2,5R", "P3,7R", "P3,8L"]
        rid, signed = cloud.build_request(targets)
        check("the Cloud signs its request", "sig" in signed)

        # 2. the robot verifies it
        mission = link.on_request(json.dumps(signed).encode())
        check("the robot accepts a properly signed request",
              mission is not None and mission.total == 6)

        forged = json.loads(json.dumps(signed))
        forged["payload"]["targets"] = ["P1,1R"]
        check("and REFUSES one whose targets were rewritten",
              link.on_request(json.dumps(forged).encode()) is None,
              "anyone on the broker could otherwise drive the robot")
        unsigned = {"payload": signed["payload"]}
        check("and refuses one with no signature at all",
              link.on_request(json.dumps(unsigned).encode()) is None)

        # 3. run it
        link.mission = mission
        link.run_mission(lambda: 0.0)

        state = cloud.state()
        check("all six reports were accepted",
              state["summary"]["accepted"] == 6,
              f"accepted {state['summary']['accepted']}, "
              f"rejected {list(cloud.rejected)}")
        check("none was rejected", state["summary"]["rejected"] == 0,
              str(list(cloud.rejected)))
        check("the store has six visits", state["summary"]["visits"] == 6)
        check("coverage counts six of forty-eight",
              state["summary"]["coverage"] == [6, 48])

        # 4. the data actually arrived intact
        measured = {s["label"]: s for s in state["stations"] if s["measured"]}
        check("every requested station is in the store",
              set(measured) == set(targets),
              f"missing {set(targets) - set(measured)}")
        check("readings are present and numeric",
              all(isinstance(v, (int, float))
                  for s in measured.values() for v in s["values"].values()))
        check("the photograph was stored as a file",
              all((storedir / s["photo"]).exists() for s in measured.values()))
        check("the QR image was stored as a file",
              all((storedir / s["qr"]).exists() for s in measured.values()))

        # 5. the injected anomaly survived the whole chain
        check("the dry patch at P2,4R arrived flagged or low",
              measured["P2,4R"]["values"]["humidity"]
              < measured["P2,5R"]["values"]["humidity"] - 10,
              "the anomaly did not survive the pipeline")

        # 6. parking was measured, not assumed
        parks = [s["parking_error_m"] for s in measured.values()]
        check("parking error is recorded for every visit",
              all(p is not None for p in parks))
        check("and is inside tolerance", max(parks) < 0.04,
              f"worst {max(parks):.3f} m")

        # 7. CSV export
        csv_path = cloud.store.export_csv()
        rows = csv_path.read_text().strip().splitlines()
        check("the CSV export has a header and one row per visit",
              len(rows) == 7, f"{len(rows)} lines")
        check("and names the quantities", "temperature" in rows[0])

        # 8. a report from an unknown robot is refused
        from agri.crypto_ecc import generate_keypair
        rogue = generate_keypair()
        rogue_visitor = Visitor(robot_id="youbot-01",
                                cloud_public_pem=cloud_keys.public_pem,
                                robot_private_pem=rogue.private_pem,
                                read_sensors=field.read,
                                driver=SimulatedDriver(seed=9))
        envelope, _ = rogue_visitor.visit("P1,1R", 0.0)
        before = cloud.accepted
        verdict = cloud.on_report(json.dumps(envelope).encode())
        check("a report signed by an unknown key is refused",
              cloud.accepted == before and verdict.startswith("REJECTED"),
              verdict)
        check("and the rejection is visible, not silent",
              len(cloud.rejected) == 1 and cloud.state()["summary"]["rejected"] == 1)


def check_numpy_abi() -> None:
    """numpy 2 next to ROS 2 Jazzy is a runtime break, not a warning.

    Every rosidl Python extension in Jazzy was compiled against numpy 1.26.
    Import numpy 2 in the same interpreter and the failure arrives when a
    message is first deserialised -- inside a callback, after the simulator
    is up. So the dependency that drags numpy 2 in is pinned, and the pin is
    checked here rather than remembered.
    """
    print("\nnumpy ABI, for the interpreter that also runs ROS")
    pyproject = (ROOT / "pyproject.toml").read_text()
    check("opencv is capped below 5, which is the version that wants numpy 2",
          "opencv-python-headless>=4.8,<5" in pyproject,
          "opencv 5 declares numpy>=2 and shadows the system numpy that "
          "rclpy's message extensions were compiled against")

    try:
        import numpy                                   # noqa: PLC0415
        import rclpy                                   # noqa: F401,PLC0415
    except ImportError:
        print("        (rclpy is absent here, so there is nothing to clash)")
        return
    check("the installed numpy matches what ROS was built against",
          int(numpy.__version__.split(".")[0]) < 2,
          f"numpy {numpy.__version__} is loaded alongside rclpy. Fix it:\n"
          "    pip install 'numpy<2' 'opencv-python-headless<5'")


def check_mqtt_compat() -> None:
    """paho-mqtt 2.0 changed the Client constructor. Both sides build one.

    This is the kind of break that never shows up in a test suite because
    the suite runs offline: everything passes, and then the broker side
    fails on the machine that has the newer paho -- which is the machine the
    demonstration is on.
    """
    print("\nMQTT client construction (paho 1.x and 2.x)")
    from agri.protocol import mqtt_client

    server = (ROOT / "agri" / "cloud" / "server.py").read_text()
    node = (ROOT / "ros2" / "src" / "agri_robot" / "agri_robot"
            / "robot_node.py").read_text()
    for name, src in (("the Cloud", server), ("the robot node", node)):
        check(f"{name} builds its client through the compatibility helper",
              "mqtt_client(" in src and "mqtt.Client(" not in src,
              "calling mqtt.Client directly breaks on paho 2.x")

    try:
        import paho.mqtt.client  # noqa: F401
    except ImportError:
        check("paho is installed, so the helper can be exercised", False,
              "pip install paho-mqtt")
        return

    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter("error")          # a warning here IS the bug
        try:
            client = mqtt_client("preflight")
            check("a client is built without a deprecation warning", True)
        except Exception as exc:                # noqa: BLE001
            check("a client is built without a deprecation warning", False,
                  f"{type(exc).__name__}: {exc}")
            return
    check("and it has the four methods the system uses",
          all(hasattr(client, m) for m in
              ("publish", "subscribe", "will_set", "connect")))


def check_aisles() -> None:
    """The route geometry, swept over every pair of stations.

    This is the check that pays for itself. A wrong headland coordinate does
    not raise, does not warn and does not collide -- the robot has no
    collision geometry -- it just drives the chassis through a gutter while
    the recording is running. Here it is 2 256 route calculations and a
    second of arithmetic.
    """
    print("\naisles: can the robot get anywhere without touching anything")
    from agri.aisles import (BANDS, HEADLAND_MARGIN, HEADLANDS, band,
                             clearance, route, route_clearance)
    from agri.catalogue import ROW_Y, all_stations

    st = all_stations()
    check("four free bands, one per aisle", len(BANDS) == 4)
    check("no band contains a gutter centre line",
          all(band(y) is None for y in ROW_Y.values()))
    check("every station is in a band",
          all(band(s.y) is not None for s in st))
    check("standing at a station clears everything by 0.16 m",
          min(clearance(s.x, s.y) for s in st) >= 0.15,
          f"worst {min(clearance(s.x, s.y) for s in st):.3f} m")

    check("the two headlands are not the same distance out",
          abs(HEADLANDS[0]) != abs(HEADLANDS[1]),
          "the boom points +x, so the sensor point is the leading edge going "
          "west and the trailing edge going east; one number cannot be right "
          "at both ends")
    check("a headland leaves a margin at both ends", HEADLAND_MARGIN > 0.05,
          f"{HEADLAND_MARGIN:.3f} m")

    worst, pair = float("inf"), None
    for a in st:
        for b in st:
            if a is not b:
                c = route_clearance(a.x, a.y, b.x, b.y)
                if c < worst:
                    worst, pair = c, (a.label, b.label)
    check(f"all {len(st) * (len(st) - 1)} station-to-station routes stay "
          "clear of the gutters and walls", worst > 0.05,
          f"worst {worst:.3f} m, on {pair[0]} -> {pair[1]}" if pair else "")
    check("and so does every route out of the dock",
          min(route_clearance(HEADLANDS[0], -1.85, s.x, s.y)
              for s in st) > 0.05)

    # The lidar brake. Every case below is one the simulator actually
    # produced or would have: the first version used a +/-35 degree cone on
    # raw ranges and stopped dead on the first metre of the first leg, for
    # the robot's own camera pedestal.
    from agri.aisles import BODY_BOX, brake_clearance   # noqa: PLC0415

    def verdict(px, py, dx, dy):
        c = brake_clearance(px, py, dx, dy)
        if c is None:
            return "ignored"
        return "self" if c < -0.02 else ("brake" if c < 0.35 else "free")

    check("the outline is asymmetric: boom in front, chassis behind",
          BODY_BOX[2] > -BODY_BOX[0],
          "a symmetric half-width brakes for things the tail clears")
    check("the camera pedestal is not mistaken for an obstacle",
          verdict(0.12, 0.0, 1, 0) == "self",
          "it crosses the lidar plane 0.12 m ahead and returns ~0.11 m")
    check("nor is the mast", verdict(0.22, 0.0, 1, 0) == "self")
    check("a gutter wall being driven PAST is not in the way",
          verdict(0.61, 0.35, 1, 0) == "ignored"
          and verdict(2.0, 0.35, 1, 0) == "ignored",
          "a cone catches the wall the route was designed to pass")
    check("a gutter end being STRAFED past is not in the way either",
          verdict(-0.37, 0.45, 0, 1) == "ignored",
          "at the east headland the tail clears it by 0.21 m")
    check("but something genuinely ahead does stop it",
          verdict(0.70, 0.0, 1, 0) == "brake")
    check("and something genuinely in the strafe path does too",
          verdict(0.10, 0.45, 0, 1) == "brake")
    check("clear floor further off does not",
          verdict(1.50, 0.0, 1, 0) == "free")

    same = route(-3.15, -1.75, 2.25, -1.75)
    check("a trip inside one aisle is a single leg", len(same) == 1, str(same))
    across = route(-3.15, -1.75, -3.15, -0.65)
    check("a trip to another aisle goes round by a headland",
          len(across) == 3 and across[0][0] in HEADLANDS, str(across))


def check_vision() -> None:
    """The floor camera, against frames rendered by its own inverse
    projection. Not a substitute for pointing it at Gazebo -- but it is the
    difference between shipping a sign error and shipping a camera."""
    print("\nthe floor camera and the red cross")
    import math                                        # noqa: PLC0415

    import numpy as np                                 # noqa: PLC0415

    from agri.catalogue import CROSS_ARM, MIN_CROSS_GAP, all_stations
    from agri.vision import NadirCamera, find_cross, render_cross

    cam = NadirCamera()
    length, width = cam.footprint()
    check("the camera sees a useful patch of floor",
          length > 0.35 and width > 0.5,
          f"{length:.2f} x {width:.2f} m")

    for dx, dy in ((0.0, 0.0), (0.10, 0.0), (0.0, -0.12), (-0.07, 0.05)):
        s = find_cross(render_cross(dx, dy))
        ok = s is not None and abs(s.dx - dx) < 0.005 and abs(s.dy - dy) < 0.005
        check(f"a cross at ({dx:+.2f}, {dy:+.2f}) is found there", ok,
              "not found" if s is None else f"reported ({s.dx:+.3f}, {s.dy:+.3f})")

    # Orientation. The whole reason the marker is a cross rather than a dot.
    errs = [abs(math.degrees(find_cross(render_cross(0, 0, math.radians(d))).yaw) - d)
            for d in (0, 8, -14, 30)]
    check("and its ORIENTATION is recovered to within a degree",
          max(errs) < 1.0, f"worst {max(errs):.2f} deg")

    # The one that matters: two stations 0.10 m apart in an inner aisle.
    # NOTE the indices. all_stations() builds fresh objects on each call,
    # so comparing across two calls with "is not" pairs every station with
    # ITSELF and reports a gap of zero -- which is exactly what the first
    # version of this check did, and it "passed" nothing.
    st = all_stations()
    tightest = min(abs(a.y - b.y)
                   for i, a in enumerate(st) for b in st[i + 1:]
                   if abs(a.x - b.x) < 1e-6)
    check("neighbouring crosses do not touch",
          tightest - 2 * CROSS_ARM >= MIN_CROSS_GAP,
          f"{tightest:.3f} m apart, arms reach {2 * CROSS_ARM:.3f} m -- they "
          "would merge into one blob and the robot would centre between them")
    for off in (0.0, 0.02, -0.03):
        s = find_cross(render_cross(0.0, off, others=((0.0, off - tightest),)))
        check(f"with a neighbour {tightest:.2f} m away, the NEARER one wins "
              f"(offset {off:+.2f})",
              s is not None and abs(s.dy - off) < 0.005,
              "not found" if s is None else f"reported {s.dy:+.3f}")

    check("an empty floor gives no sighting",
          find_cross(np.zeros((240, 320, 3), np.uint8)) is None)
    solid = np.zeros((240, 320, 3), np.uint8)
    solid[:, :] = (38, 64, 54)
    solid[90:160, 120:200] = (232, 26, 26)
    check("a solid red blob is not mistaken for a cross",
          find_cross(solid) is None,
          "the fill-ratio test is what separates a marker from a stray "
          "red object")
    edge = render_cross(0.30, 0.0)      # half of it outside the frame
    check("a cross clipped by the frame edge is refused, not averaged",
          find_cross(edge) is None,
          "its centroid is biased inwards and the robot would stop short")


def check_ros_package() -> None:
    """The ROS package cannot be imported here -- there is no rclpy on this
    machine -- so it is checked as TEXT. Every one of these caught something
    that would otherwise have shown up as a robot sitting still, saying
    nothing, waiting for a topic no one publishes."""
    print("\nthe ROS 2 package (read, not imported: no rclpy here)")
    import ast                                         # noqa: PLC0415

    pkg = ROOT / "ros2" / "src" / "agri_robot"
    driver = (pkg / "agri_robot" / "driver.py").read_text()
    node = (pkg / "agri_robot" / "robot_node.py").read_text()
    launch = (pkg / "launch" / "agri.launch.py").read_text()
    setup = (pkg / "setup.py").read_text()
    bridge = (pkg / "config" / "gz_bridge_agri.yaml").read_text()

    for name, src in (("driver", driver), ("robot_node", node),
                      ("launch", launch), ("setup", setup)):
        try:
            ast.parse(src)
            check(f"{name}.py is valid Python", True)
        except SyntaxError as exc:
            check(f"{name}.py is valid Python", False, str(exc))

    # Every topic the driver touches must be bridged, and vice versa. This
    # is the failure that looks like a hang: the node subscribes to /odom,
    # nothing publishes it, and the robot waits forever in silence.
    bridged = set(re.findall(r'ros_topic_name:\s*"/([^"]+)"', bridge))
    used = set(re.findall(r'create_(?:publisher|subscription)\(\s*\w+,\s*"([^"]+)"',
                          driver))
    check("every topic the driver uses is in the bridge config",
          used <= bridged, f"missing from the bridge: {sorted(used - bridged)}")
    # /clock is bridged for use_sim_time and read by rclpy, not by driver.py.
    check("and the bridge carries nothing nobody reads",
          bridged - used <= {"clock"},
          f"bridged but unused: {sorted(bridged - used - {'clock'})}")

    from agri.world.make_robot import FLOOR_CAM_TOPIC   # noqa: PLC0415
    check("the floor camera's gz topic matches the URDF generator",
          f'gz_topic_name: "/{FLOOR_CAM_TOPIC}"' in bridge)
    world = (ROOT / "worlds" / "greenhouse_cloud.sdf").read_text()
    name = re.search(r'<world name="([^"]+)"', world).group(1)
    check("the joint-state bridge names the world the launch file opens",
          f"/world/{name}/model/youbot/joint_state" in bridge,
          f"the world is {name!r}; a mismatch means the head angle never "
          "arrives and photograph() waits out its timeout at every station")

    check("the node is exposed as a console script",
          "robot_node = agri_robot.robot_node:main" in setup)

    # colcon runs exactly this assertion at BUILD time:
    #   AssertionError: 'data_files' must be relative, '/...' is absolute
    # so an absolute path here is not a subtly wrong install, it is a
    # package that does not build. Run setup.py with setup() stubbed out and
    # check every entry, which is cheaper than owning a ROS installation.
    import runpy                                       # noqa: PLC0415
    import setuptools                                  # noqa: PLC0415

    captured: dict = {}
    real_setup, cwd = setuptools.setup, os.getcwd()
    setuptools.setup = lambda **kw: captured.update(kw)
    try:
        os.chdir(pkg)
        runpy.run_path("setup.py", run_name="__main__")
    finally:
        setuptools.setup = real_setup
        os.chdir(cwd)

    entries = [(dest, src) for dest, srcs in captured.get("data_files", [])
               for src in srcs]
    check("setup.py declares some data_files at all", bool(entries))
    absolute = [s for _, s in entries if os.path.isabs(s)]
    check("every data_files path is relative, as colcon requires",
          not absolute, f"absolute: {absolute}")
    missing = [s for _, s in entries if not (pkg / s).exists()]
    check("and every one of them exists", not missing, f"missing: {missing}")
    installed = {os.path.basename(s) for _, s in entries}
    check("the generated world and robot are among them",
          {"greenhouse_cloud.sdf", "youbot_agri.urdf"} <= installed,
          f"only {sorted(installed)}")
    check("the launch file installs and uses the GENERATED assets",
          "greenhouse_cloud.sdf" in launch and "youbot_agri.urdf" in launch
          and "worlds" in setup and "urdf" in setup)
    check("the launch file spawns the base 0.50 m behind the headland",
          '"-x", "-4.58"' in launch,
          "the sensor point must land on HEADLAND_WEST = -4.08")
    check("shutdown reuses the Phase B killer rather than a second one",
          "kill_sim.sh" in launch)

    # Both of these were computed correctly and then never applied. A
    # variable that is assigned and not used reads as finished work, and
    # neither failure names itself: the meshes silently do not render, and
    # the node dies on an import a second after the simulator opens.
    check("the resource path is actually SET, not merely computed",
          'SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", res_path)' in launch,
          "without it gz cannot resolve model://youbot_gazebo/meshes/*.stl "
          "and the robot renders as nothing")
    check("the robot node is given a PYTHONPATH",
          'additional_env={"PYTHONPATH": py_path}' in launch,
          "colcon writes the console script with the SYSTEM python's shebang, "
          "so the node does not inherit a virtual environment and cannot "
          "import agri")
    # Nothing else in this file may be computed and dropped.
    assigned = set(re.findall(r"^    (\w+) = ", launch, re.M))
    unused = sorted(n for n in assigned
                    if len(re.findall(rf"\b{n}\b", launch)) < 2)
    check("no other value in the launch file is computed and then dropped",
          not unused, f"assigned but never read: {unused}")

    # The driver must not quietly re-derive geometry that is tested elsewhere.
    # The driver must not re-derive geometry that is tested in agri.aisles.
    # Matched loosely on purpose: the first version pinned the exact import
    # line and broke the moment a second name was imported from the module,
    # which is a check failing for a reason that is not a defect.
    check("the driver takes its geometry from agri.aisles",
          re.search(r"^from agri\.aisles import .*\broute\b", driver, re.M)
          and re.search(r"^from agri\.aisles import .*\bbrake_clearance\b",
                        driver, re.M),
          "route() and brake_clearance() are swept over every station pair "
          "and every self-hit case by the suite; a private copy in the "
          "driver would be tested by nothing")
    check("and does not keep a private copy of the headland coordinates",
          "HEADLAND" not in driver.split('"""')[2])
    from agri.catalogue import all_stations                 # noqa: PLC0415
    st = all_stations()
    tightest = min(abs(a.y - b.y) for i, a in enumerate(st)
                   for b in st[i + 1:] if abs(a.x - b.x) < 1e-6)
    cap = _const(driver, "VISUAL_MAX_CORRECTION")
    check("vision can never move the robot far enough to reach a neighbour",
          cap < tightest / 2,
          f"the cap is {cap:.3f} m and the nearest other station is "
          f"{tightest:.3f} m away: a bad sighting could park the robot closer "
          "to the neighbour than to the station it is filing under")
    check("the node says out loud that the readings are synthesised",
          "SYNTHESISED" in node)


def _const(src: str, name: str) -> float:
    m = re.search(rf"^{name}\s*=\s*([-\d.]+)", src, re.M)
    if not m:
        raise AssertionError(f"{name} not found")
    return float(m.group(1))


def check_demo() -> None:
    """The offline demo is the thing that gets run in front of people, so it
    is the thing most worth having a check on. Running it as a SUBPROCESS is
    deliberate: it is the only way to exercise argument parsing, the console
    entry point and the label splitter, which is where a demo actually
    breaks."""
    print("\nthe offline demo, run as a subprocess")
    import subprocess                                  # noqa: PLC0415

    with tempfile.TemporaryDirectory() as tmp:
        r = subprocess.run(
            [sys.executable, "-m", "agri.demo", "P1,1R,P2,4R,P3,7L",
             "--work", tmp, "--explain"],
            cwd=ROOT, capture_output=True, text=True, timeout=180,
            env={**os.environ, "PYTHONPATH": str(ROOT)})
        out = r.stdout + r.stderr
        check("it exits cleanly", r.returncode == 0, out[-1500:])
        check("the comma inside a label is not read as a separator",
              "asks for 3 station(s)" in out and "P2,4R" in out, out[:400])
        check("all three reports were accepted",
              "3 report(s) accepted, 0 rejected" in out, out[-900:])
        check("--explain shows the ciphertext, not the readings",
              "ECIES-P256-HKDF-SHA256-AES256GCM" in out
              and "sealed.ciphertext" in out, out[-900:])
        check("and replays both refusals",
              "REJECTED from youbot-01" in out
              and "the robot refused and did not move" in out, out[-900:])
        # An "attack demo" that quietly stopped attacking would still print
        # a reassuring refusal, so check the hole message never appears.
        check("the demo does not claim a refusal it did not get",
              "that is a hole" not in out)


def check_hygiene() -> None:
    print("\nrepository hygiene")
    gitignore = (ROOT / ".gitignore").read_text()
    check("private keys are gitignored",
          "keys/" in gitignore and "*.pem" in gitignore,
          "a repository that has held a private key has published it")
    tracked = list(ROOT.rglob("*.pem"))
    check("and none is present in the tree", not tracked,
          f"found {tracked}")
    check("the Phase B project is untouched",
          (ROOT.parent / "src" / "youbot_control").is_dir()
          and (ROOT.parent / "scripts" / "check_regressions.py").exists(),
          "the harvesting robot must keep working")


def main() -> int:
    print("cloud_agri pre-flight checks")
    for fn in (check_labels, check_catalogue, check_crypto, check_qr,
               check_sensors, check_numpy_abi, check_mqtt_compat, check_aisles,
               check_vision,
               check_world,
               check_ros_package, check_end_to_end, check_demo,
               check_hygiene):
        fn()
    print()
    if FAILURES:
        print(f"  {len(FAILURES)} of {CHECKS} checks FAILED:")
        for f in FAILURES:
            print(f"    - {f}")
        return 1
    print(f"  all {CHECKS} checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
