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

import csv
import json
import os
import random
import re
import sys
import tempfile
import threading
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
    # The brief asks the Cloud for "the data of a specific plant", and a
    # plant is not a station: it has a measurement point on each side. A
    # system that could only be asked for P2,5R would be answering a
    # different question from the one it was given.
    from agri.labels import expand_target                # noqa: PLC0415
    from agri.protocol import expand_targets             # noqa: PLC0415

    check("a plant means BOTH of its stations",
          expand_target("P2,5") == ["P2,5R", "P2,5L"],
          str(expand_target("P2,5")))
    check("in the same order as a full sweep visits them",
          all_labels()[:2] == expand_target("P1,1"))
    check("a station still means just that station",
          expand_target("P2,5R") == ["P2,5R"])
    check("a plant is accepted through the Cloud's request path",
          expand_targets("P3,8") == ["P3,8R", "P3,8L"])
    check("plants and stations can be mixed in one request",
          expand_targets(["P2,5", "P1,1R"])
          == ["P2,5R", "P2,5L", "P1,1R"])
    check("and asking for a plant twice does not visit it twice",
          expand_targets(["P2,5", "P2,5R"]) == ["P2,5R", "P2,5L"])
    refuses("a plant that does not exist is refused too",
            lambda: expand_target("P4,1"), LabelError)

    refuses("row 4 does not exist", lambda: normalise("P4,1R"), LabelError)
    refuses("plant 9 does not exist", lambda: normalise("P1,9R"), LabelError)
    refuses("side M does not exist", lambda: normalise("P1,1M"), LabelError)
    refuses("empty is not a label", lambda: normalise(""), LabelError)


def check_catalogue() -> None:
    print("\ncatalogue geometry")
    import math

    from agri.catalogue import (BASE_HALF_WIDTH, CAMERA_LEVER_X,
                                GUTTER_HALF_WIDTH, ROW_Y, SENSOR_OFFSET_X,
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

    # THE PAN ANGLE, CHECKED AGAINST THE PLANT RATHER THAN AGAINST 90.
    #
    # These three used to assert the head turns exactly +/-90 degrees. That
    # is a statement about the CROSS -- the plant is square across the aisle
    # from it -- and the camera is not on the cross. It sits
    # CAMERA_LEVER_X = 0.32 m behind, on the mast, so the true bearing is
    # 108.1 degrees and the old assertion was passing on a value that
    # missed the plant by 30.2 degrees at every station. With a 69-degree
    # field of view the plant stayed just inside the frame, which is why
    # 48 photographs a run went on looking acceptable.
    #
    # The property worth pinning is not a number. It is: WHEREVER the head
    # ends up pointing, the plant is in front of the lens. So the check now
    # projects a ray from the lens along the commanded angle and asserts it
    # hits the plant.
    def aiming_error_deg(s, yaw):
        cx, cy = s.camera_position(yaw)
        want = math.atan2(s.plant_y - cy, s.plant_x - cx)
        got = s.pan_for(yaw) + yaw          # world angle the lens looks along
        d = math.atan2(math.sin(want - got), math.cos(want - got))
        return abs(math.degrees(d))

    for lab in ("P2,1R", "P2,1L", "P1,4R", "P3,8L"):
        s = station(lab)
        for yaw, way in ((0.0, "east"), (math.pi, "west")):
            check(f"the lens is aimed AT the plant at {lab} driving {way}",
                  aiming_error_deg(s, yaw) < 1e-6,
                  f"{aiming_error_deg(s, yaw):.1f} deg off; the camera is on "
                  "the mast, not on the cross")

    # And the two sides still look opposite ways, which is the thing the
    # +/-90 assertion was really protecting.
    check("an R station and an L station look opposite ways",
          station("P2,1R").pan_for(0.0) * station("P2,1L").pan_for(0.0) < 0)
    check("and reversing the direction of travel flips the head",
          station("P2,1R").pan_for(0.0) * station("P2,1R").pan_for(math.pi) < 0,
          "otherwise half the visits photograph the opposite row")

    # The head has to be able to REACH the angle. Phase B limited it to 90
    # degrees, which Phase C's geometry exceeds; make_robot.py widens it,
    # and this is what would catch that widening being lost.
    from agri.world.make_robot import PAN_LIMIT                # noqa: PLC0415
    worst = max(abs(s.pan_for(0.0)) for s in st)
    check("every station's pan angle is inside the joint's travel",
          worst < PAN_LIMIT,
          f"needs {math.degrees(worst):.1f} deg, joint allows "
          f"{math.degrees(PAN_LIMIT):.1f} deg -- the head would saturate and "
          "the plant would sit off-centre in frame, looking fine")

    # THE ROBOT IS AT THE PLANT, not half a metre down the aisle from it.
    # This is what moving the crosses by SENSOR_OFFSET_X bought, and it is
    # invisible from any other check: the old geometry parked the chassis
    # 0.50 m along the row and every reading was still filed correctly.
    for s in st:
        base_x = s.x - SENSOR_OFFSET_X
        if abs(base_x - s.plant_x) > 1e-9:
            check(f"the chassis stands beside plant {s.label}", False,
                  f"base_link at x={base_x:.2f}, plant at x={s.plant_x:.2f}")
            break
    else:
        check("the chassis stands beside the plant, at all 48 stations", True)
    # THE PARKED CHASSIS COVERS THE MARK, COMPLETELY.
    #
    # A floor fiducial belongs under the robot. The camera that reads it
    # cannot be under the robot -- the chassis blocks the ground -- so the
    # mark is read on the way in, with the boom over it, and the robot then
    # advances the boom length onto it. These two checks pin the geometry at
    # both ends of that sequence; drop either and the sequence still runs
    # and still parks somewhere plausible.
    from agri.catalogue import BASE_HALF_LENGTH          # noqa: PLC0415
    from agri.world.make_world import CROSS_ARM as _ARM  # noqa: PLC0415
    exposed = []
    for s in st:
        cx, cy = s.cross
        base_x = s.x - SENSOR_OFFSET_X
        if (abs(cx - base_x) + _ARM > BASE_HALF_LENGTH
                or abs(cy - s.y) + _ARM > BASE_HALF_WIDTH):
            exposed.append(s.label)
    check("the parked chassis hides every painted mark, entirely",
          not exposed,
          f"{exposed[:4]} stick out from under the robot")
    check("and the mark the camera reads is the one it parks on",
          station("P2,4R").fix_pose == station("P2,4R").cross,
          "the fix pose and the mark must be the same point, or the robot "
          "corrects against one thing and parks on another")
    check("the park pose is exactly one boom length past the mark",
          abs((station("P2,4R").x - station("P2,4R").cross[0])
              - SENSOR_OFFSET_X) < 1e-9)

    drv = (ROOT / "ros2" / "src" / "agri_robot" / "agri_robot"
           / "driver.py").read_text()
    check("the driver routes to the FIX pose, not to the park pose",
          "s.fix_pose" in drv and "route(x, y, fx, fy)" in drv)
    check("and only advances after the visual fix has run",
          drv.index("self._centre_on_cross(s)")
          < drv.index("onto the mark"),
          "advancing first would put the chassis over the mark and leave "
          "the camera correcting against empty floor")
    check("the advance is measured from where the fix left the robot",
          "px + SENSOR_OFFSET_X" in drv,
          "advancing to an absolute x would throw the correction away in "
          "the last half metre")

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


def check_replay() -> None:
    print("\nreplay: a signed order stops being an order")
    from datetime import datetime, timedelta, timezone

    from agri.replay import (FRESHNESS_S, ReplayError, ReplayGuard,
                             parse_stamp)

    T0 = datetime(2026, 8, 9, 12, 0, 0, tzinfo=timezone.utc)

    def at(seconds: float) -> datetime:
        return T0 + timedelta(seconds=seconds)

    def stamp(dt) -> str:
        return dt.strftime("%Y-%m-%dT%H:%M:%SZ")

    g = ReplayGuard(window_s=120.0)
    age = g.check("aaaa1111", stamp(T0), now=at(3))
    check("a fresh order is accepted", abs(age - 3.0) < 1e-6, f"age {age} s")
    check("and its age is reported, not merely approved", age > 0)

    refuses("the SAME order a second time is refused",
            lambda: g.check("aaaa1111", stamp(T0), now=at(4)), ReplayError)
    check("a DIFFERENT order at the same instant is still accepted",
          g.check("bbbb2222", stamp(T0), now=at(4)) is not None,
          "the guard must reject repeats, not throttle the fleet")

    refuses("an order older than the window is refused",
            lambda: g.check("cccc3333", stamp(T0), now=at(121)), ReplayError)
    check("an order at exactly the window edge is still obeyed",
          ReplayGuard(120.0).check("dddd4444", stamp(T0), now=at(120)) is not None,
          "an off-by-one here refuses work at the boundary, intermittently")

    # A future-dated order cannot be a replay -- nobody captures a message
    # before it is sent -- so it must be diagnosed as a clock, not an attack.
    try:
        ReplayGuard(120.0).check("eeee5555", stamp(at(600)), now=T0)
        check("a future-dated order is refused", False, "it was accepted")
    except ReplayError as exc:
        check("a future-dated order is refused", True)
        check("and is blamed on the CLOCK, not on an attacker",
              "FUTURE" in str(exc) and "timedatectl" in str(exc),
              str(exc).splitlines()[0])
    try:
        ReplayGuard(120.0).check("ffff6666", stamp(T0), now=at(9000))
        check("a stale order is refused", False, "it was accepted")
    except ReplayError as exc:
        msg = str(exc)
        check("a stale order is refused", True)
        check("and the refusal says HOW stale, so it is diagnosable",
              "9000 s" in msg and "120 s" in msg, msg.splitlines()[0])
        check("and offers both readings: a clock, or an actual replay",
              "timedatectl" in msg and "replayed" in msg)

    # The skew warning must fire BEFORE the day the drift starts refusing.
    g2 = ReplayGuard(120.0)
    check("a comfortable age produces no warning",
          g2.suspicious_skew(3.0) == "")
    check("but an age eating half the window does",
          "clock skew" in g2.suspicious_skew(70.0),
          "a drift should be visible on the day it is harmless")

    # The set must not grow without bound over a long campaign: ids older
    # than the window are refused on age anyway, so keeping them buys
    # nothing.
    g3 = ReplayGuard(window_s=60.0)
    for i in range(200):
        g3.check(f"id{i:04d}", stamp(at(i)), now=at(i))
    check("the seen-set is pruned to the window, not grown forever",
          g3.remembered <= 61, f"{g3.remembered} ids after 200 requests")
    check("and a pruned id is refused on AGE, so pruning opens no hole",
          _raises(lambda: g3.check("id0000", stamp(T0), now=at(199))),
          "this is why the two halves are needed together")

    refuses("a request with no date at all is refused",
            lambda: ReplayGuard().check("gggg7777", ""), ReplayError)
    refuses("and so is one with a date in the wrong format",
            lambda: parse_stamp("09/08/2026 12:00"), ReplayError)

    check("the default window is stated in seconds, not guessed",
          FRESHNESS_S == 120.0, f"{FRESHNESS_S} s")


def _raises(fn) -> bool:
    try:
        fn()
    except Exception:                                # noqa: BLE001
        return True
    return False


def _find_all(text: str, needle: str) -> list[int]:
    out, i = [], text.find(needle)
    while i != -1:
        out.append(i)
        i = text.find(needle, i + 1)
    return out


def _chain_links(path: Path) -> bool:
    """Every line names the hash of the one before it."""
    prev = None
    for line in path.read_text().splitlines():
        rec = json.loads(line)
        if prev is not None and rec.get("prev") != prev:
            return False
        prev = rec.get("hash")
    return prev is not None


def check_trust() -> None:
    print("\ntrust: naming a key, and retiring one")
    import subprocess                                     # noqa: PLC0415

    from agri import keys as K                            # noqa: PLC0415
    from agri.crypto_ecc import generate_keypair          # noqa: PLC0415
    from agri.trust import (MAX_AGE_DAYS, RevocationError,  # noqa: PLC0415
                            TrustStore, age_days, age_warning,
                            expiry_hint, fingerprint, meta_path,
                            record_birth, summarise)

    a, b = generate_keypair(), generate_keypair()
    fa = fingerprint(a.public_pem)
    check("a key gets a short readable name", len(fa) == 19 and fa.count(":") == 3,
          fa)
    check("the same key always gets the same name",
          fingerprint(a.public_pem) == fa)
    check("and two keys do not share one", fingerprint(b.public_pem) != fa)
    check("a trailing newline does not rename a key",
          fingerprint(a.public_pem + b"\n") == fa,
          "hashing the PEM TEXT would give one key two names and send "
          "somebody hunting a substitution that never happened")
    check("and neither does re-wrapping it",
          fingerprint(a.public_pem.decode()) == fa)

    # --- the list --------------------------------------------------------
    d = Path(tempfile.mkdtemp()) / "k"
    st = TrustStore(d)
    check("an empty trust store refuses nothing", st.entries() == {})
    st.check(a.public_pem, "robot")
    check("and lets a good key through", True)

    st.revoke(a.public_pem, "laptop stolen")
    check("a revoked key is remembered by fingerprint",
          st.entries().get(fa, "").startswith("laptop stolen"),
          str(st.entries()))
    try:
        st.check(a.public_pem, "robot")
        check("and is refused", False, "it was accepted")
    except RevocationError as exc:
        check("and is refused", True)
        check("with the reason, so the operator knows what happened",
              "laptop stolen" in str(exc) and fa in str(exc),
              str(exc).splitlines()[0])
    st.check(b.public_pem, "robot")
    check("a different key is still fine", True)
    st.revoke(a.public_pem, "again")
    check("revoking twice does not duplicate the line",
          list(st.entries()).count(fa) == 1)
    check("the file says out loud that nothing distributes it",
          "does not tell the other machine" in st.path.read_text(),
          "a revocation list mistaken for a PKI is worse than none")

    # --- age -------------------------------------------------------------
    d2 = Path(tempfile.mkdtemp()) / "k2"
    d2.mkdir(parents=True)
    record_birth("robot", a.public_pem, d2)
    check("a generated key records when it was born",
          meta_path("robot", d2).exists() and age_days("robot", d2) < 1)
    check("and the file says it is NOT a certificate",
          "not a certificate" in meta_path("robot", d2).read_text(),
          "an unsigned date the key holder can edit is a reminder, and "
          "calling it more than that would be a lie")
    check("a fresh key produces no nagging", age_warning("robot", d2) == "")
    meta = json.loads(meta_path("robot", d2).read_text())
    meta["created"] = "2020-01-01T00:00:00Z"
    meta_path("robot", d2).write_text(json.dumps(meta))
    w = age_warning("robot", d2)
    check("an old one does, with the command that fixes it",
          "--rotate robot" in w and "days" in w, w)
    check("and the summary line carries the fingerprint and the age",
          fingerprint(a.public_pem) in summarise("robot", a.public_pem, d2)
          and "days old" in summarise("robot", a.public_pem, d2))
    check("a key with no recorded birthday is not an error",
          age_days("cloud", d2) is None
          and "unknown" in expiry_hint(None),
          "a working deployment must not stop for a bookkeeping file")
    check("the rotation horizon is a year, stated not guessed",
          MAX_AGE_DAYS == 365)

    # --- rotation --------------------------------------------------------
    d3 = Path(tempfile.mkdtemp()) / "k3"
    K.bootstrap(d3)
    before = K.public_of("robot", d3, generate=False)
    old_fp, new_fp = K.rotate("robot", d3, "decommissioned")
    after = K.public_of("robot", d3, generate=False)
    check("rotation replaces the key", after != before and new_fp != old_fp)
    check("and revokes the old one in the same breath",
          old_fp in TrustStore(d3).entries(),
          "a rotation that leaves the old key valid has retired nothing")
    check("and the new one is NOT revoked", new_fp not in TrustStore(d3).entries())
    archived = sorted((d3 / "retired").glob("*/robot_private.pem"))
    check("the old PRIVATE key is archived, never destroyed",
          len(archived) == 1 and archived[0].read_bytes(),
          "reports already sealed to it can be opened with nothing else; "
          "deleting it makes them unreadable forever")
    check("and the archived private key is not world-readable",
          oct(archived[0].stat().st_mode)[-3:] == "600",
          oct(archived[0].stat().st_mode))
    check("the cloud key was left alone",
          K.public_of("cloud", d3, generate=False)
          not in (b"",) and len(TrustStore(d3).entries()) == 1,
          "rotating one role must not retire the other")
    refuses("rotating a role that has no key is refused, not invented",
            lambda: K.rotate("robot", Path(tempfile.mkdtemp()) / "empty"),
            FileNotFoundError)

    # --- the programs actually refuse ------------------------------------
    from agri.cloud.server import Cloud                   # noqa: PLC0415
    from agri.cloud.store import Store                    # noqa: PLC0415
    d4 = Path(tempfile.mkdtemp())
    K.bootstrap(d4 / "k")
    Cloud(Store(d4 / "s"), d4 / "k")
    check("a Cloud starts with good keys", True)
    TrustStore(d4 / "k").revoke(
        K.public_of("robot", d4 / "k", generate=False), "test")
    try:
        Cloud(Store(d4 / "s"), d4 / "k")
        check("and REFUSES TO START once the robot key is revoked", False,
              "it started anyway")
    except RevocationError:
        check("and REFUSES TO START once the robot key is revoked", True,
              "a warning about a retired key is a warning nobody acts on")
    node = (ROOT / "ros2" / "src" / "agri_robot" / "agri_robot"
            / "robot_node.py").read_text()
    check("and so does the robot node",
          "store.check(self.cloud_public" in node,
          "otherwise a retired Cloud key still drives the robot")

    # --- the command line ------------------------------------------------
    d5 = Path(tempfile.mkdtemp()) / "k5"
    r = subprocess.run([sys.executable, "-m", "agri.keys", "--dir", str(d5)],
                       capture_output=True, text=True, cwd=str(ROOT))
    check("generating prints the fingerprints to compare",
          r.returncode == 0
          and fingerprint(K.public_of("cloud", d5, generate=False)) in r.stdout,
          r.stdout[-200:])
    check("and says why comparing them matters",
          "different keys" in r.stdout,
          "every message refused with nothing saying which key is the "
          "failure this line prevents")
    r = subprocess.run([sys.executable, "-m", "agri.keys", "--dir", str(d5),
                        "--revoke", "robot", "--reason", "sold"],
                       capture_output=True, text=True, cwd=str(ROOT))
    check("--revoke works from the command line and says to restart",
          r.returncode == 0 and "RESTART" in r.stdout, r.stdout[-200:])
    check("and says the file has to be copied across",
          "other machine" in r.stdout)
    r = subprocess.run([sys.executable, "-m", "agri.keys", "--dir", str(d5),
                        "--list"], capture_output=True, text=True,
                       cwd=str(ROOT))
    check("--list shows what is trusted and what is not",
          "1 revoked key" in r.stdout and "sold" in r.stdout, r.stdout[-200:])


def check_broker_auth() -> None:
    print("\nbroker: TLS, credentials, and no silent downgrade")
    import argparse                                       # noqa: PLC0415
    import os                                             # noqa: PLC0415

    from agri.protocol import (DEFAULT_PORT, PASSWORD_ENV,  # noqa: PLC0415
                               TLS_PORT, BrokerAuth, ProtocolError,
                               add_broker_arguments, broker_auth_from)

    class FakeClient:
        """Records what was asked of it, so 'we set TLS' can be checked."""

        def __init__(self):
            self.tls_args = None
            self.insecure = False
            self.creds = None

        def tls_set(self, **kw):
            self.tls_args = kw

        def tls_insecure_set(self, v):
            self.insecure = v

        def username_pw_set(self, u, p=None):
            self.creds = (u, p)

    d = Path(tempfile.mkdtemp())
    ca, crt, key = (d / "ca.crt", d / "c.crt", d / "c.key")
    for f in (ca, crt, key):
        f.write_text("not a real certificate")

    # --- the default: plain, and it SAYS plain ---------------------------
    plain, c = BrokerAuth(), FakeClient()
    line = plain.apply(c)
    check("with no flags the broker link is plaintext", not plain.tls)
    check("and the program says so out loud",
          "PLAINTEXT" in line and "no authentication" in line, line)
    check("and nothing was configured on the client", c.tls_args is None)
    check("the plain default port is 1883", plain.port(None) == DEFAULT_PORT)

    # --- TLS -------------------------------------------------------------
    tls = BrokerAuth(ca=str(ca))
    c = FakeClient()
    line = tls.apply(c)
    check("naming a CA is what turns TLS on", tls.tls)
    check("and the port follows to 8883 without being asked twice",
          tls.port(None) == TLS_PORT,
          "two things to change is one thing to forget")
    check("an explicit port still wins", tls.port(1884) == 1884)
    check("the certificate is REQUIRED, not merely offered",
          c.tls_args and c.tls_args["cert_reqs"].name == "CERT_REQUIRED",
          str(c.tls_args))
    check("and the line printed names TLS", "TLS" in line, line)
    check("hostname checking is on unless it is switched off", not c.insecure)

    c = FakeClient()
    BrokerAuth(ca=str(ca), insecure=True).apply(c)
    check("--broker-insecure is honoured", c.insecure)
    check("and is loud about it in the printed line",
          "HOSTNAME NOT CHECKED" in BrokerAuth(ca=str(ca),
                                               insecure=True).apply(FakeClient()),
          "a certificate nobody checks the name on is decorative")

    # --- client certificates ---------------------------------------------
    c = FakeClient()
    both = BrokerAuth(ca=str(ca), cert=str(crt), key=str(key))
    line = both.apply(c)
    check("a client certificate is passed through with its key",
          c.tls_args["certfile"] == str(crt)
          and c.tls_args["keyfile"] == str(key), str(c.tls_args))
    check("and is named in the line", "client certificate" in line, line)
    refuses("a certificate with no key is refused",
            lambda: BrokerAuth(ca=str(ca), cert=str(crt)).apply(FakeClient()),
            ProtocolError)
    refuses("and a key with no certificate",
            lambda: BrokerAuth(ca=str(ca), key=str(key)).apply(FakeClient()),
            ProtocolError)
    refuses("a path that does not exist is refused before connecting",
            lambda: BrokerAuth(ca=str(d / "nope.crt")).apply(FakeClient()),
            ProtocolError)

    # --- the refusal that matters ----------------------------------------
    try:
        BrokerAuth(username="cloud", password="hunter2").apply(FakeClient())
        check("a password with NO TLS is refused", False, "it was accepted")
    except ProtocolError as exc:
        check("a password with NO TLS is refused", True)
        check("and the refusal says the password would go out in clear",
              "CLEAR TEXT" in str(exc), str(exc).splitlines()[0])
    c = FakeClient()
    BrokerAuth(ca=str(ca), username="cloud", password="hunter2").apply(c)
    check("with TLS the credentials are set", c.creds == ("cloud", "hunter2"))

    # --- the password never comes from argv ------------------------------
    ap = argparse.ArgumentParser()
    add_broker_arguments(ap)
    flags = {a.option_strings[0] for a in ap._actions if a.option_strings}
    check("there is NO --broker-password flag, by design",
          "--broker-password" not in flags and "--broker-pass" not in flags,
          f"/proc/<pid>/cmdline is world-readable; found {sorted(flags)}")
    check("and the help says where the password does come from",
          PASSWORD_ENV in ap.format_help(), PASSWORD_ENV)

    saved = os.environ.get(PASSWORD_ENV)
    os.environ[PASSWORD_ENV] = "from-the-environment"
    try:
        args = ap.parse_args(["--broker-ca", str(ca), "--broker-user", "cloud"])
        auth = broker_auth_from(args)
        check("the password is read from the environment",
              auth.password == "from-the-environment")
        c = FakeClient()
        auth.apply(c)
        check("and reaches the client", c.creds == ("cloud",
                                                    "from-the-environment"))
        del os.environ[PASSWORD_ENV]
        auth = broker_auth_from(ap.parse_args(
            ["--broker-ca", str(ca), "--broker-user", "cloud"]))
        line = auth.apply(FakeClient())
        check("and a MISSING password is said out loud, not passed as empty",
              "NO password" in line and PASSWORD_ENV in line,
              "connecting anonymously while believing you authenticated is "
              "the failure this exists to make impossible")
    finally:
        if saved is None:
            os.environ.pop(PASSWORD_ENV, None)
        else:
            os.environ[PASSWORD_ENV] = saved

    # --- both programs must offer the same flags -------------------------
    srv = (ROOT / "agri" / "cloud" / "server.py").read_text()
    node = (ROOT / "ros2" / "src" / "agri_robot" / "agri_robot"
            / "robot_node.py").read_text()
    check("the Cloud takes the broker flags",
          "add_broker_arguments(ap)" in srv and "broker_auth_from(args)" in srv)
    check("and the robot node takes the same settings as parameters",
          all(f'declare_parameter("{p}"' in node
              for p in ("broker_ca", "broker_cert", "broker_key",
                        "broker_user", "broker_insecure")),
          "a TLS Cloud and a plaintext robot is two programs that cannot "
          "talk, diagnosed as a broken broker")
    check("neither falls back to plaintext when TLS was asked for",
          "return 2" in srv and "raise SystemExit" in node,
          "a silent downgrade is worse than no TLS: somebody believes it")
    check("the reference broker config is in the repository",
          (ROOT / "deploy" / "mosquitto.conf").exists()
          and (ROOT / "deploy" / "acl").exists())
    conf = (ROOT / "deploy" / "mosquitto.conf").read_text()
    check("and it does not leave the plain port listening",
          "listener 8883" in conf and "listener 1883" not in conf,
          "a broker still answering on 1883 is a broker every client can "
          "still reach in the clear")
    check("and it requires a client certificate, not just a server one",
          "require_certificate true" in conf
          and "allow_anonymous false" in conf)
    acl = (ROOT / "deploy" / "acl").read_text()
    check("the ACL stops a robot publishing as another robot",
          "pattern write agri/v1/status/%u" in acl,
          "%u is the certificate's own name; without it one node can "
          "report another as dead")
    check("and stops any robot issuing orders at all",
          "topic write agri/v1/request" in acl
          and "pattern write agri/v1/request" not in acl,
          "the day one node is compromised, this is the difference "
          "between one bad node and the fleet")


def check_plant_camera() -> None:
    print("\nthe plant camera: which way is up, and what is in frame")
    import math                                           # noqa: PLC0415

    from agri.catalogue import (PLANT_BASE_Z,  # noqa: PLC0415
                                PLANT_CAM_H, PLANT_CAM_W, PLANT_CAM_Z,
                                PLANT_FRAME_FILL, PLANT_HEIGHT_M,
                                PLANT_SPREAD_M, plant_camera_distance,
                                plant_camera_frame, plant_frame_span)
    from agri.envelope import PARK_TOLERANCE              # noqa: PLC0415
    from agri.world.plants import (PLANT_WIDTH_M,  # noqa: PLC0415
                                   fitted_height, needs_roll, up_axis)

    # --- WHICH WAY IS UP -------------------------------------------------
    # glTF fixes +Y as up in its specification. fit() assumed Z for every
    # format because the STLs came first, so every plant lay on its side --
    # invisible in a render of a rosette, and given away only by the one
    # mesh with a base plate baked in, which stood up as a dark wall in the
    # middle of every photograph.
    check("a glTF is read as Y-up, per its own specification",
          up_axis(Path("x.glb")) == 1 and up_axis(Path("x.gltf")) == 1)
    check("and an STL as Z-up, which is what these ones are",
          up_axis(Path("x.stl")) == 2)
    check("only the Y-up formats get rolled upright",
          needs_roll(Path("x.glb")) and not needs_roll(Path("x.stl")),
          "Gazebo does not rotate a glTF for you")

    glbs = sorted((ROOT / "meshes").glob("strawberry_[0-9].glb"))
    if glbs:
        heights = {m.name: fitted_height(m) for m in glbs}
        check("every fitted plant is WIDER than it is tall, like a rosette",
              all(h < PLANT_WIDTH_M for h in heights.values()),
              {k: round(v, 3) for k, v in heights.items()})
        tallest = max(heights.values())
        check("PLANT_HEIGHT_M matches the tallest mesh actually installed",
              abs(PLANT_HEIGHT_M - tallest) < 0.02,
              f"catalogue says {PLANT_HEIGHT_M}, the meshes say "
              f"{tallest:.3f} -- the camera is aimed from that number")

    check("the catalogue and the mesh fitter agree on how wide a plant is",
          abs(PLANT_SPREAD_M - PLANT_WIDTH_M) < 1e-9,
          f"{PLANT_SPREAD_M} vs {PLANT_WIDTH_M}")

    # The rejected mesh must stay rejected, and stay explained.
    rej = sorted((ROOT / "meshes" / "rejected").glob("*.glb"))
    check("the mesh with a baked base plate is out of the rotation",
          rej and all("plate" in r.name for r in rej),
          f"{[r.name for r in rej]} -- the plate is welded into the same "
          "primitive as the plant, so it cannot be dropped as a node")
    check("and it is not picked up by the greenhouse's glob",
          not any(r.parent == ROOT / "meshes" for r in rej))

    # --- WHAT IS IN FRAME ------------------------------------------------
    d = plant_camera_distance()
    pitch, hfov = plant_camera_frame()
    vfov = 2 * math.atan(math.tan(hfov / 2) * PLANT_CAM_H / PLANT_CAM_W)
    fb, pb, pt, ft = plant_frame_span()

    check("the camera looks UP at a plant standing above it",
          pitch < 0 and PLANT_BASE_Z + PLANT_HEIGHT_M > PLANT_CAM_Z,
          f"pitch {math.degrees(-pitch):+.1f} deg nose-up")
    check("the whole plant is inside the frame, top and bottom",
          fb < pb and pt < ft,
          f"frame {math.degrees(fb):+.1f}..{math.degrees(ft):+.1f}, "
          f"plant {math.degrees(pb):+.1f}..{math.degrees(pt):+.1f}")

    # The margin has to survive the worst parking the Cloud will accept.
    park = math.degrees(math.atan2(PARK_TOLERANCE, d))
    for name, margin in (("below", math.degrees(pb - fb)),
                         ("above", math.degrees(ft - pt))):
        check(f"and survives the worst allowed parking error, {name}",
              margin > park,
              f"{margin:.1f} deg of margin against {park:.1f} deg for a "
              f"{PARK_TOLERANCE * 100:.0f} cm error")
    half_plant = math.degrees(math.atan2(PLANT_SPREAD_M / 2, d))
    check("and across, which is the tight direction for a rosette",
          math.degrees(hfov) / 2 - half_plant > park,
          f"{math.degrees(hfov) / 2 - half_plant:.1f} deg vs {park:.1f}")

    # The framing must be worth doing at all.
    fw, fh = 2 * d * math.tan(hfov / 2), 2 * d * math.tan(vfov / 2)
    check("the plant is a majority of the frame's width",
          PLANT_SPREAD_M / fw > 0.5,
          f"{PLANT_SPREAD_M / fw * 100:.0f}% -- Phase B's 68.8 deg lens "
          f"gave 31%, which is why two thirds of every photograph was wall")
    check("but not so much that it fills it",
          PLANT_SPREAD_M / fw < PLANT_FRAME_FILL + 0.02,
          f"{PLANT_SPREAD_M / fw * 100:.0f}%")
    check("the lens is narrower than the one it replaces",
          hfov < 1.2, f"{math.degrees(hfov):.1f} deg against 68.8")

    # Move the plants and the camera must follow, or none of this holds.
    import agri.catalogue as C                             # noqa: PLC0415
    saved = C.PLANT_BASE_Z
    try:
        C.PLANT_BASE_Z = 0.0                    # plants grown on the floor
        floor_pitch, _ = C.plant_camera_frame()
        check("moving the plants to the floor re-aims the camera DOWN",
              floor_pitch > 0,
              f"{math.degrees(-floor_pitch):+.1f} deg -- the pitch is "
              "computed from PLANT_BASE_Z, so the aim follows the plants "
              "instead of having to be retuned by hand")
    finally:
        C.PLANT_BASE_Z = saved

    # And the generator has to write it into the robot.
    urdf = (ROOT / "urdf" / "youbot_agri.urdf")
    if not urdf.exists():
        from agri.world import ensure as _e                # noqa: PLC0415
        _e.ensure()
    text = urdf.read_text()
    check("the generated robot carries the computed pitch, not Phase B's",
          f"rpy=\"0 {pitch:.6f} 0\"" in text and "-0.28 0" not in text,
          "Phase B aimed this camera at crates down an aisle")
    check("and the two copies of that pose agree",
          text.count(f"{pitch:.6f}") >= 2,
          "the joint and the <sensor> both carry it; a disagreement means "
          "the picture and the tf frame point different ways")
    check("and the narrowed lens",
          f"<horizontal_fov>{hfov:.6f}</horizontal_fov>" in text)
    check("while the floor camera keeps its own wide lens",
          "<horizontal_fov>1.2</horizontal_fov>" in text,
          "it looks at a 4 cm cross 45 cm below it and needs the width")


def check_on_the_cross() -> None:
    """One point, named the same way by the world, RViz and the report."""
    print("\nparking: the robot stands ON the mark, and everything says so")
    import re as _re                                       # noqa: PLC0415

    from agri.catalogue import (BASE_HALF_LENGTH,  # noqa: PLC0415
                                BASE_HALF_WIDTH, CROSS_ARM, SENSOR_OFFSET_X,
                                all_stations, station)
    from agri.envelope import PARK_TOLERANCE, build_report  # noqa: PLC0415
    from agri.measurement import Measurement               # noqa: PLC0415
    from agri.world import ensure as _ensure               # noqa: PLC0415

    # 1. THE THREE POINTS ARE THREE POINTS, and only one of them is paint.
    s = station("P2,5R")
    check("the mark, the chassis and the boom tip are named separately",
          s.cross == s.park_pose and s.sensor_pose == (s.x, s.y)
          and s.cross != s.sensor_pose,
          f"cross {s.cross}, parked {s.park_pose}, sensor {s.sensor_pose}")
    check("and the boom tip finishes exactly one boom past the mark",
          abs((s.sensor_pose[0] - s.cross[0]) - SENSOR_OFFSET_X) < 1e-9)

    # 2. THE CHASSIS COVERS THE PAINT. At every station, with the whole
    #    cross inside the footprint and not merely its centre.
    worst = 0.0
    for st in all_stations():
        bx, by = st.park_pose
        along = BASE_HALF_LENGTH - CROSS_ARM
        across = BASE_HALF_WIDTH - CROSS_ARM
        worst = max(worst, CROSS_ARM + abs(st.cross[0] - bx),
                    CROSS_ARM + abs(st.cross[1] - by))
        if along < 0 or across < 0:
            break
    check("the parked chassis covers the WHOLE cross, at all 48 stations",
          all(abs(st.cross[0] - st.park_pose[0]) + CROSS_ARM
              <= BASE_HALF_LENGTH
              and abs(st.cross[1] - st.park_pose[1]) + CROSS_ARM
              <= BASE_HALF_WIDTH for st in all_stations()),
          f"chassis {2 * BASE_HALF_LENGTH:.2f} x {2 * BASE_HALF_WIDTH:.2f} m, "
          f"cross arm {CROSS_ARM} m")
    check("and with room to spare for the worst allowed parking error",
          BASE_HALF_WIDTH - CROSS_ARM > PARK_TOLERANCE,
          f"{BASE_HALF_WIDTH - CROSS_ARM:.3f} m of margin across against "
          f"{PARK_TOLERANCE} m of tolerance")

    # 3. THE PAINT IN THE WORLD IS AT .cross.
    _ensure.ensure()
    world = (ROOT / "worlds" / "greenhouse_cloud.sdf").read_text()
    painted = {}
    for m in _re.finditer(r'<model name="(cross_[^"]+)">\s*<static>true</static>'
                          r'\s*<pose>([-\d.]+) ([-\d.]+)', world):
        painted[m.group(1)] = (float(m.group(2)), float(m.group(3)))
    check("the world paints one cross per station", len(painted) == 48,
          f"{len(painted)} found")
    if painted:
        want = {(round(st.cross[0], 3), round(st.cross[1], 3))
                for st in all_stations()}
        got = {(round(x, 3), round(y, 3)) for x, y in painted.values()}
        check("and paints them where .cross says, not where .x says",
              got == want,
              f"{len(got - want)} painted somewhere else")

    # 4. RVIZ DRAWS THE SAME POINT. This is the one that was wrong: the
    #    markers were at s.x, so a robot parked exactly on its mark
    #    appeared to stop half a metre short of every cross -- in the view
    #    an evaluator watches.
    viz = (ROOT / "ros2" / "src" / "agri_robot" / "agri_robot"
           / "viz_node.py").read_text()
    check("RViz draws its markers on the PAINT, not on the boom target",
          "cx, cy = s.cross" in viz and "_seg(s.x - CROSS_ARM" not in viz,
          "markers at s.x put every cross 0.50 m ahead of the painted one")

    # 5. THE REPORT SAYS THE SAME POINT.
    m = Measurement(label="P2,5R",
                    values={"temperature": 21.4, "humidity": 63.2,
                            "luminosity": 12480, "co2": 431, "ph": 6.42})
    rep = build_report(m, b"\xff\xd8jpg", (4, 4), "youbot-01")["station"]
    check("the report's cross is the painted cross",
          (rep["cross"]["x"], rep["cross"]["y"]) == s.cross,
          f"{rep['cross']} vs {s.cross}")
    check("and it also says where the chassis and the boom tip end up",
          (rep["parked_at"]["x"], rep["parked_at"]["y"]) == s.park_pose
          and (rep["sensor_at"]["x"], rep["sensor_at"]["y"]) == s.sensor_pose)
    check("and the four points are not silently the same",
          len({tuple(rep[k].values())
               for k in ("cross", "sensor_at", "plant_at")}) == 3)

    # 6. THE DASHBOARD SHOWS THE SAME POINT, AND SHOWS THE ROBOT ON IT.
    from agri.cloud.server import Cloud                    # noqa: PLC0415
    from agri.cloud.store import Store                     # noqa: PLC0415
    d = Path(tempfile.mkdtemp())
    cl = Cloud(Store(d / "s"), d / "k")
    api = {r["label"]: r for r in cl.state()["stations"]}
    check("the dashboard is sent the PAINTED cross as x, y",
          (api["P2,5R"]["x"], api["P2,5R"]["y"]) == s.cross,
          f"{api['P2,5R']['x'], api['P2,5R']['y']} vs {s.cross}")
    check("and the boom target separately, so neither has to be inferred",
          (api["P2,5R"]["sensor_x"], api["P2,5R"]["sensor_y"])
          == s.sensor_pose)

    js = (ROOT / "agri" / "cloud" / "dashboard" / "app.js").read_text()
    css = (ROOT / "agri" / "cloud" / "dashboard" / "style.css").read_text()
    check("the map draws the robot at all",
          "function robotLayer" in js and "svg += robotLayer(st)" in js,
          "48 crosses and no robot meant 'did it park on the mark' was a "
          "question only Gazebo could answer")
    check("and draws the CHASSIS where the chassis is, not the boom tip",
          "node.pose.x - BOOM * Math.cos(yaw)" in js,
          "the status pose is the sensor point; drawing the base there "
          "would put the robot half a metre past every mark it is on")
    check("the station it is standing on is highlighted",
          'class="parked"' in js and ".parked line" in css)
    check("in a hue nothing else on the map uses",
          "#ffd166" in css and "#ffb020" in css)
    check("but only when it is genuinely ON it",
          "bestD <= 0.12" in js,
          "highlighting the nearest mark from 30 cm away would say the "
          "opposite of the truth")
    check("and the offset is given as a NUMBER, in millimetres",
          "mm off" in js and "S.parked.d * 1000" in js,
          "the highlight says which mark; the evaluation is about how far")

    # 7. THE DRIVER'S TWO STAGES STILL ADD UP TO base_link ON THE MARK.
    drv = (ROOT / "ros2" / "src" / "agri_robot" / "agri_robot"
           / "driver.py").read_text()
    check("the driver reads the mark with the boom, then advances onto it",
          "s.fix_pose" in drv and "px + SENSOR_OFFSET_X" in drv
          and drv.index("_centre_on_cross(s)") < drv.index("px + SENSOR"),
          "correcting AFTER the advance would throw the fix away")
    check("and Driver.pose() is the sensor point, which is what makes that "
          "arithmetic land on the mark",
          "The SENSOR point, which is what a station names" in drv,
          "if pose() were base_link the same code would park a boom "
          "length past every cross")


def check_textures() -> None:
    print("\ntextures: a 2 MB file that costs half a gigabyte")
    import shutil                                          # noqa: PLC0415

    from agri.world.plants import build_meshes             # noqa: PLC0415
    from agri.world.textures import (MAX_TEXTURE_PX,  # noqa: PLC0415
                                     gpu_megabytes, oversized, shrink,
                                     texture_sizes)

    glbs = sorted((ROOT / "meshes").glob("strawberry_[0-9].glb"))
    if not glbs:
        check("the strawberry meshes are present", False)
        return

    # THE ONE THAT KILLED A DEMONSTRATION. Three meshes, two 4096x4096
    # textures each: 536 MB of GPU memory before a single triangle, on a
    # laptop whose integrated graphics take that out of system RAM. Gazebo
    # loaded so slowly the launch declared the bridge broken, and then the
    # kernel killed it.
    for m in glbs:
        check(f"{m.name} is within the texture budget",
              not oversized(m),
              f"{texture_sizes(m)} -- {gpu_megabytes(m):.0f} MB of GPU "
              f"memory for one plant")
    total = sum(gpu_megabytes(m) for m in glbs)
    check("the whole greenhouse's textures fit in a laptop",
          total < 32, f"{total:.0f} MB across {len(glbs)} meshes")
    check("the budget is stated, not implied", MAX_TEXTURE_PX == 512,
          f"{MAX_TEXTURE_PX} px against the ~320 the plant ever occupies")

    # Shrinking must not damage the mesh: same triangles, same bounds,
    # still self-contained. Rebuilding the binary chunk moves every
    # bufferView, and one wrong offset is a mesh that loads as garbage.
    from agri.world.plants import (glb_bounds,  # noqa: PLC0415
                                   glb_is_self_contained, triangle_count)
    d = Path(tempfile.mkdtemp())
    src = glbs[0]
    copy = d / src.name
    shutil.copy(src, copy)
    before = (triangle_count(copy), glb_bounds(copy))
    check("shrinking an already-small mesh is a no-op",
          "already within" in shrink(copy),
          "re-encoding on every run would churn the file and the git diff")
    check("and leaves the geometry untouched",
          (triangle_count(copy), glb_bounds(copy)) == before)
    check("and it still carries its images",
          glb_is_self_contained(copy),
          "rebuilding the buffer with one wrong offset loses them silently")

    # A mesh that is over budget must be REFUSED, not warned about: the
    # failure it causes is an OOM kill during load, which says nothing
    # about textures.
    big = _fat_texture_glb(d)
    check("a 4K texture is detected", bool(oversized(big)),
          str(texture_sizes(big)))
    world = d / "w.sdf"
    from agri.world.make_world import (build as build_world,  # noqa: PLC0415
                                       default_source as _src)
    build_world(_src(), world)
    try:
        build_meshes(world, [big], 0.80, 1)
        check("and building a world with it is REFUSED", False,
              "it was accepted")
    except ValueError as exc:
        check("and building a world with it is REFUSED", True)
        check("with the memory figure, not just a complaint",
              "MB of GPU memory" in str(exc), str(exc)[:120])
        check("and the command that fixes it",
              "agri.world.textures" in str(exc), str(exc)[-120:])

    # And it has to shrink for real.
    n_before = gpu_megabytes(big)
    shrink(big)
    check("shrinking actually shrinks it",
          gpu_megabytes(big) < n_before / 10,
          f"{n_before:.0f} -> {gpu_megabytes(big):.0f} MB")
    check("and the result is within budget", not oversized(big))


def _fat_texture_glb(d: Path) -> Path:
    """A minimal GLB carrying one 4096x4096 texture, for the refusal test."""
    import io                                              # noqa: PLC0415
    import json as _j                                      # noqa: PLC0415
    import struct                                          # noqa: PLC0415

    from PIL import Image                                  # noqa: PLC0415

    buf = io.BytesIO()
    Image.new("RGB", (4096, 4096), (30, 90, 30)).save(buf, "JPEG", quality=20)
    jpeg = buf.getvalue()
    pos = struct.pack("<9f", 0, 0, 0, 1, 0, 0, 0, 1, 0)
    idx = struct.pack("<3H", 0, 1, 2) + b"\0\0"
    blob = pos + idx + jpeg
    js = {
        "asset": {"version": "2.0"},
        "buffers": [{"byteLength": len(blob)}],
        "bufferViews": [
            {"buffer": 0, "byteOffset": 0, "byteLength": len(pos)},
            {"buffer": 0, "byteOffset": len(pos), "byteLength": len(idx)},
            {"buffer": 0, "byteOffset": len(pos) + len(idx),
             "byteLength": len(jpeg)},
        ],
        "accessors": [
            {"bufferView": 0, "componentType": 5126, "count": 3,
             "type": "VEC3", "min": [0, 0, 0], "max": [1, 1, 0]},
            {"bufferView": 1, "componentType": 5123, "count": 3,
             "type": "SCALAR"},
        ],
        "images": [{"bufferView": 2, "mimeType": "image/jpeg"}],
        "textures": [{"source": 0}],
        "materials": [{"pbrMetallicRoughness": {
            "baseColorTexture": {"index": 0}}}],
        "meshes": [{"primitives": [
            {"attributes": {"POSITION": 0}, "indices": 1, "material": 0}]}],
        "nodes": [{"mesh": 0}],
        "scenes": [{"nodes": [0]}],
    }
    jb = _j.dumps(js, separators=(",", ":")).encode()
    jb += b" " * (-len(jb) % 4)
    bb = blob + b"\0" * (-len(blob) % 4)
    out = struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(jb) + 8 + len(bb))
    out += struct.pack("<II", len(jb), 0x4E4F534A) + jb
    out += struct.pack("<II", len(bb), 0x004E4942) + bb
    p = d / "fat.glb"
    p.write_bytes(out)
    return p


def check_ensure() -> None:
    print("\nthe generated world: you cannot forget to build it")
    import shutil                                         # noqa: PLC0415
    import time as _t                                     # noqa: PLC0415

    from agri.world import ensure as E                    # noqa: PLC0415

    d = Path(tempfile.mkdtemp()) / "cloud_agri"
    (d / "worlds").mkdir(parents=True)
    (d / "urdf").mkdir()
    (d / "meshes").mkdir()
    (d / "agri" / "world").mkdir(parents=True)
    for src in E.WORLD_SOURCES + E.URDF_SOURCES:
        dst = d / src
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(ROOT / src, dst)

    world = d / "worlds" / "greenhouse_cloud.sdf"
    urdf = d / "urdf" / "youbot_agri.urdf"

    why = E.why_stale(d)
    check("a fresh clone with no world is reported stale",
          any("does not exist" in r for r in why), str(why))
    check("and so is the missing robot description",
          any("youbot_agri.urdf does not exist" in r for r in why), str(why))

    world.write_text("<sdf><world name='x'></world></sdf>")
    urdf.write_text("<robot/>")
    _t.sleep(0.01)
    check("with both present and nothing newer, nothing is stale",
          E.why_stale(d) == [], str(E.why_stale(d)))

    # The one that moved the crosses. A pull rewrites the .py files with a
    # fresh mtime; a world built before it still has the old marks, and the
    # robot parks perfectly on paint that is no longer there.
    (d / "agri" / "catalogue.py").touch()
    check("a world older than the catalogue is stale",
          any("older than the code" in r for r in E.why_stale(d)),
          str(E.why_stale(d)))

    world.touch()
    urdf.touch()
    check("and touching it settles that", E.why_stale(d) == [])

    # No plants. This is the exact state a demonstration was given: a world
    # built by make_world alone, so every plant renders as a green sphere.
    (d / "meshes" / "strawberry_1.glb").write_bytes(b"glTF fake")
    why = E.why_stale(d)
    check("a world with meshes available but none used is stale",
          any("green sphere" in r for r in why), str(why))
    check("and the reason says what it would LOOK like, not just what is "
          "missing", any("render as a green sphere" in r for r in why),
          "the operator sees spheres; the message has to connect to that")

    # The invisible one: a mesh path from somebody else's disk.
    world.write_text(
        "<sdf><world name='x'><model><visual><geometry><mesh>"
        "<uri>file:///home/someone-else/strawberry_1.glb</uri>"
        "</mesh></geometry></visual></model></world></sdf>")
    dead = E.dead_mesh_uris(world)
    check("a mesh URI pointing at another machine is detected",
          dead == ["/home/someone-else/strawberry_1.glb"], str(dead))
    check("and it is reported as stale, with the path",
          any("do not exist on this machine" in r and "someone-else" in r
              for r in E.why_stale(d)), str(E.why_stale(d)))
    check("and the reason says Gazebo would be SILENT about it",
          any("say nothing about it" in r for r in E.why_stale(d)),
          "a missing texture or a dead path renders as nothing, with no "
          "error anywhere -- that is why this check exists at all")

    real = d / "meshes" / "strawberry_1.glb"
    world.write_text("<sdf><world name='x'><model><visual><geometry><mesh>"
                     f"<uri>file://{real}</uri>"
                     "</mesh></geometry></visual></model></world></sdf>")
    check("a mesh path that DOES exist here is fine",
          E.dead_mesh_uris(world) == [] and E.why_stale(d) == [],
          str(E.why_stale(d)))

    # And the real thing, in the real tree.
    E.ensure()
    check("the real world ends up current", E.why_stale() == [],
          str(E.why_stale()))
    text = (ROOT / "worlds" / "greenhouse_cloud.sdf").read_text()
    check("with all 24 plants meshed", text.count("<mesh>") == 24)
    check("and every mesh path it names is on THIS disk",
          E.dead_mesh_uris(ROOT / "worlds" / "greenhouse_cloud.sdf") == [])
    check("running it again is a no-op", E.ensure() == [],
          "a generator that rebuilds on every launch adds a minute to "
          "every demonstration")

    # run_sim.sh has to actually call it, or none of this happens.
    sh = (ROOT / "run_sim.sh").read_text()
    # Anchored on the real invocation, not on "ros2 launch": a comment
    # forty lines up mentions the phrase, and matching that made this pass
    # while measuring nothing. Same trap as the ordering checks below.
    check("run_sim.sh builds the world before launching",
          "agri.world.ensure" in sh
          and sh.index("agri.world.ensure")
          < sh.index("stdbuf -oL -eL ros2 launch"),
          "generating it after the launch reads it is the same as not "
          "generating it")
    check("and stops if it cannot", "die \"could not generate the world" in sh)
    ignore = (ROOT / ".gitignore").read_text()
    check("the generated files are NOT tracked",
          "worlds/" in ignore and "urdf/" in ignore,
          "a tracked generated file makes every regeneration block the "
          "next git pull -- which is how a demonstration ran four commits "
          "behind without anybody noticing")
    import subprocess                                     # noqa: PLC0415
    tracked = subprocess.run(
        ["git", "ls-files", "worlds", "urdf"], cwd=str(ROOT),
        capture_output=True, text=True).stdout.strip()
    check("and git agrees", not tracked, tracked)


def check_chain() -> None:
    print("\nstore: a filed reading that changes says so")
    import subprocess

    from agri.cloud.store import (GENESIS, Store,  # noqa: PLC0415
                                  line_hash, verify_chain)
    from agri.envelope import build_report                # noqa: PLC0415
    from agri.measurement import Measurement              # noqa: PLC0415

    def file_one(st, label: str, temp: float = 21.4) -> None:
        m = Measurement(label=label,
                        values={"temperature": temp, "humidity": 63.2,
                                "luminosity": 12480, "co2": 431, "ph": 6.42})
        st.add_report(build_report(m, b"\xff\xd8jpg", (4, 4), "youbot-01"))

    d = Path(tempfile.mkdtemp())
    st = Store(d / "s")
    check("an empty store's tip is genesis, not an empty string",
          st.tip == GENESIS,
          "an empty string would make a truncated file look like a fresh one")

    for i, lab in enumerate(["P1,1R", "P1,1L", "P1,2R"]):
        file_one(st, lab, 20.0 + i)

    checked, problems = st.verify()
    check("three filed readings chain cleanly", checked == 3 and not problems,
          str(problems))
    check("and the tip moved off genesis", st.tip != GENESIS)
    check("each line names the hash of the one before it",
          _chain_links(st.jsonl),
          "without prev, reordering the file would go unnoticed")

    # THE ACTUAL ATTACK: change one number in one filed line. Everything
    # upstream verified this reading -- signature, seal, QR against the
    # numbers, pose against the catalogue -- and then wrote it in plain text.
    lines = st.jsonl.read_text().splitlines()
    rec = json.loads(lines[1])
    rec["values"]["temperature"] = 99.9
    lines[1] = json.dumps(rec, sort_keys=True)
    st.jsonl.write_text("\n".join(lines) + "\n")

    _, problems = verify_chain(st.jsonl)
    check("editing a filed temperature is DETECTED", bool(problems),
          "the pipeline protects the reading in flight; without this it "
          "abandons it on landing")
    check("and the report names the line and says it was edited",
          any("line 2" in p and "EDITED" in p for p in problems),
          str(problems))
    check("and ONLY that line, so the report points at the damage",
          len(problems) == 1, str(problems))

    # Now the forger does it properly: edits the line AND recomputes its
    # hash, so line 2 is internally consistent again. This is the cascade,
    # and it is the reason a hash chain is worth more than a per-line
    # checksum -- line 3 now points at a hash that no longer exists.
    rec = json.loads(lines[1])
    rec["hash"] = line_hash(rec, rec["prev"])
    lines[1] = json.dumps(rec, sort_keys=True)
    st.jsonl.write_text("\n".join(lines) + "\n")
    _, problems = verify_chain(st.jsonl)
    check("repairing the edited line's own hash just moves the break down",
          any("line 3" in p and "inserted, removed or reordered" in p
              for p in problems),
          "a convincing forgery has to rewrite every line to the end of "
          "the file, not one")

    # Deleting a line is the other half: every remaining line still hashes
    # to itself, so only `prev` catches it.
    d2 = Path(tempfile.mkdtemp())
    st2 = Store(d2 / "s")
    for i, lab in enumerate(["P1,1R", "P1,1L", "P1,2R"]):
        file_one(st2, lab, 20.0 + i)
    keep = st2.jsonl.read_text().splitlines()
    st2.jsonl.write_text(keep[0] + "\n" + keep[2] + "\n")
    _, problems = verify_chain(st2.jsonl)
    check("REMOVING a filed reading is detected as well",
          any("inserted, removed or reordered" in p for p in problems),
          "each line hashes itself, so a deletion leaves every remaining "
          "line internally valid -- prev is what catches it")

    # A campaign filed before the chain existed is data, not an attack.
    d3 = Path(tempfile.mkdtemp())
    (d3 / "s").mkdir(parents=True)
    legacy = d3 / "s" / "measurements.jsonl"
    legacy.write_text(json.dumps(
        {"label": "P1,1R", "timestamp": "2026-01-01T00:00:00Z",
         "robot": "youbot-01", "values": {}}) + "\n")
    _, problems = verify_chain(legacy)
    check("an unchained legacy line is reported, not called tampering",
          len(problems) == 1 and "carry no hash" in problems[0]
          and "Not tampering" in problems[0],
          "a verifier that cried wolf over old data would be ignored")
    file_one(Store(d3 / "s"), "P1,2L")
    _, problems = verify_chain(legacy)
    check("and appending to a legacy file still works",
          not any("EDITED" in p or "reordered" in p for p in problems),
          str(problems))

    # The hash must cover the CONTENT, not merely exist.
    rec = {"label": "P1,1R", "values": {"temperature": 20.0}}
    h1 = line_hash(rec, GENESIS)
    # Found by pyflakes, not by a test: one dict literal named two
    # different values "plant", and Python resolves that last-wins in
    # silence. The report carried coordinates where the plant index
    # belonged for as long as the format existed.
    from agri.catalogue import station as _station        # noqa: PLC0415
    rep = build_report(Measurement(label="P2,5R",
                                   values={"temperature": 21.4,
                                           "humidity": 63.2,
                                           "luminosity": 12480,
                                           "co2": 431, "ph": 6.42}),
                       b"\xff\xd8jpg", (4, 4), "youbot-01")
    st = rep["station"]
    check("the report's plant number is a NUMBER, beside its row",
          st["plant"] == _station("P2,5R").plant and isinstance(st["row"], int),
          f"row={st['row']!r} plant={st['plant']!r}")
    check("and the plant's position has a name of its own",
          st["plant_at"] == {"x": _station("P2,5R").plant_x,
                             "y": _station("P2,5R").plant_y},
          str(st))
    check("the cross and the plant are not the same point",
          st["cross"] != st["plant_at"],
          "the robot parks on one and photographs the other")

    check("the hash is canonical: key order does not change it",
          line_hash({"values": {"temperature": 20.0}, "label": "P1,1R"},
                    GENESIS) == h1,
          "otherwise re-serialising a file would look like tampering")
    check("but changing a value does",
          line_hash({"label": "P1,1R", "values": {"temperature": 20.1}},
                    GENESIS) != h1)
    check("and so does moving the line in the chain",
          line_hash(rec, "somethingelse") != h1)

    # It has to be runnable, not merely callable.
    r = subprocess.run([sys.executable, "-m", "agri.cloud.store", "--verify",
                        "--root", str(d2 / "s")],
                       capture_output=True, text=True, cwd=str(ROOT))
    check("--verify reports a broken chain and exits non-zero",
          r.returncode == 1 and "PROBLEM" in r.stdout, r.stdout[-200:])
    r = subprocess.run([sys.executable, "-m", "agri.cloud.store", "--verify",
                        "--root", str(d / "nothing-here")],
                       capture_output=True, text=True, cwd=str(ROOT))
    check("and says so plainly when there is nothing filed",
          r.returncode == 0 and "nothing filed" in r.stdout, r.stdout[-200:])

    d4 = Path(tempfile.mkdtemp())
    file_one(Store(d4 / "s"), "P1,1R", 21.0)
    r = subprocess.run([sys.executable, "-m", "agri.cloud.store", "--verify",
                        "--root", str(d4 / "s")],
                       capture_output=True, text=True, cwd=str(ROOT))
    check("an intact chain exits zero and prints the tip",
          r.returncode == 0 and "intact" in r.stdout and "tip" in r.stdout,
          r.stdout[-200:])
    check("and says why the tip is worth keeping off the machine",
          "other than this machine" in r.stdout,
          "a chain on the same disk as its data is a consistency check, "
          "not an integrity control, and claiming otherwise is worse than "
          "claiming nothing")


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
    from agri.catalogue import all_stations, plant_position
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
            # s.cross, not s.x. The mark goes under the parked chassis, a
            # boom length behind the pose the robot drives its sensor point
            # to; painting it at s.x leaves it in front of the robot.
            cx, cy = s.cross
            if not m or abs(float(m.group(1)) - cx) > 1e-3 \
                    or abs(float(m.group(2)) - cy) > 1e-3:
                drift.append(s.label)
        check("every painted cross is where the catalogue says", not drift,
              f"{drift[:5]} differ -- the robot would stop beside them")
        refuses("regenerating from an already-generated world is refused",
                lambda: build(target, Path(d) / "again.sdf"), ValueError)

        # --- swapping the sphere blobs for a real strawberry mesh --------
        # The mesh is an ASSET: it comes from outside the project, and
        # nothing here can assert its licence or its polygon count. So the
        # swap is a separate generator, and the property worth checking is
        # that it degrades safely -- a missing mesh must leave a world that
        # still builds, not one that will not load.
        from agri.world.plants import build as swap_plants   # noqa: PLC0415

        fake = Path(d) / "strawberry.dae"
        fake.write_text("")                 # a real .dae is not needed here
        before = target.read_text()
        report = swap_plants(target, fake, scale=0.01, z=0.80, every_nth=1)
        after = target.read_text()

        check("the mesh swap replaces every plant", "24 plant(s)" in report,
              report)
        check("and says so with the mesh it used", "strawberry.dae" in report)
        try:
            xml.dom.minidom.parseString(after)
            check("the world is still well-formed XML after the swap", True)
        except Exception as exc:                     # noqa: BLE001
            check("the world is still well-formed XML after the swap",
                  False, str(exc))
        check("the 48 crosses survive the swap",
              after.count('name="cross_') == 48)
        check("collision stays a PRIMITIVE, never the mesh",
              after.count('name="stem"') == 24 and "<mesh>" not in
              after.split('name="stem"')[0].rsplit("<collision", 1)[-1],
              "a mesh collision asks the physics engine to test every "
              "triangle on every contact query, 24 times, every millisecond")

        # The plants must not MOVE. The catalogue decides where they are and
        # the crosses are placed from it; a swap that nudged a pose would
        # put every station a few centimetres off its plant, silently.
        moved = []
        for r in range(3):
            for i in range(8):
                m = re.search(rf'<model name="plant_{r}_{i}">.*?'
                              r'<pose>([-\d.]+) ([-\d.]+)', after, re.S)
                px, py = plant_position(r + 1, i + 1)
                if not m or abs(float(m.group(1)) - px) > 1e-6 \
                        or abs(float(m.group(2)) - py) > 1e-6:
                    moved.append(f"plant_{r}_{i}")
        check("and no plant moves by a millimetre", not moved, str(moved[:4]))

        refuses("a missing mesh is refused with the path, not a traceback",
                lambda: swap_plants(target, Path(d) / "nope.dae", 1.0, 0.8, 1),
                FileNotFoundError)
        refuses("and swapping twice is refused rather than nested",
                lambda: swap_plants(target, fake, 1.0, 0.8, 1), ValueError)
        # --every-nth IS the render budget, so it is exercised rather than
        # read: 24 detailed plants plus a GPU lidar plus two cameras is
        # where the frame rate goes on integrated graphics, and an option
        # that quietly placed all 24 anyway would be worse than none.
        sparse = Path(d) / "sparse.sdf"
        build(default_source(), sparse)
        swap_plants(sparse, fake, 0.01, 0.80, 3)
        n = sparse.read_text().count("<mesh>")
        check("--every-nth places the mesh on a subset only", n == 8,
              f"{n} mesh plants with --every-nth 3; expected 8 of 24")
        check("and the other sixteen keep their blobs",
              sparse.read_text().count('<model name="plant') == 24,
              "the rows must still read as a full greenhouse")
        check("the world without any mesh at all still builds",
              "<mesh>" not in before,
              "a missing asset must degrade to the old plants, never to a "
              "world that will not load")


        # --- the real strawberry meshes, measured rather than guessed ----
        from agri.world.plants import (PLANT_WIDTH_M,  # noqa: PLC0415
                                       build_meshes, fit, stl_bounds)
        stls = sorted((ROOT / "meshes").glob("strawberry_*.stl"))
        check("the strawberry meshes are in the repository", len(stls) >= 2,
              f"found {[m.name for m in stls]}")
        if len(stls) >= 2:
            for m in stls:
                scale, lift, tris = fit(m, PLANT_WIDTH_M)
                lo, hi = stl_bounds(m)
                wide = max(hi[0] - lo[0], hi[1] - lo[1]) * scale
                check(f"{m.name} scales to a plant-sized plant",
                      abs(wide - PLANT_WIDTH_M) < 1e-6,
                      f"{wide * 100:.0f} cm across")
                check(f"{m.name} is lifted so its base rests on the surface",
                      abs(lo[2] * scale + lift) < 1e-9,
                      "these meshes are centred on their own origin; "
                      "without the lift every plant sinks by half its height")
                check(f"{m.name} fits the render budget",
                      tris < 20000, f"{tris:,} triangles x 24 plants")

            meshed = Path(d) / "meshed.sdf"
            build(default_source(), meshed)
            rep = build_meshes(meshed, stls, 0.80, 1)
            mtext = meshed.read_text()
            check("every plant gets a mesh", mtext.count("<mesh>") == 24, rep)
            check("and the meshes ALTERNATE, so the rows are not 24 copies",
                  all(mtext.count(m.name) > 0 for m in stls)
                  and max(mtext.count(m.name) for m in stls) < 24,
                  {m.name: mtext.count(m.name) for m in stls})
            check("each plant is rotated, deterministically",
                  len(set(re.findall(
                      r"<pose>0 0 [\d.]+ [\d.]+ 0 ([\d.]+)</pose>",
                                     mtext))) > 4)
            try:
                xml.dom.minidom.parseString(mtext)
                check("the meshed world is well-formed XML", True)
            except Exception as exc:             # noqa: BLE001
                check("the meshed world is well-formed XML", False, str(exc))
            check("the crosses survive the mesh swap",
                  mtext.count('name="cross_') == 48)
            check("collision is STILL a primitive, never the mesh",
                  mtext.count('name="stem"') == 24
                  and "<mesh>" not in mtext.split("<collision")[1][:400],
                  "a 7 000-triangle collision shape, 24 times, at 1 ms steps")
            check("the mesh plants carry a material, so they are not white",
                  mtext.count("<material>") >= 24,
                  "STL carries no colour of its own")

            again = Path(d) / "again.sdf"
            build(default_source(), again)
            build_meshes(again, stls, 0.80, 1)
            check("regenerating gives a byte-identical world",
                  again.read_text() == mtext,
                  "a rebuild that churns the file makes every diff useless")
            refuses("swapping meshes twice is refused",
                    lambda: build_meshes(meshed, stls, 0.80, 1), ValueError)
            refuses("and an ASCII STL is refused with the fix, not misread",
                    lambda: stl_bounds(_ascii_stl(Path(d))), ValueError)

        # --- the textured GLB plants, which is what actually ships -------
        from agri.world.plants import (glb_bounds,  # noqa: PLC0415
                                       glb_is_self_contained, mesh_bounds,
                                       triangle_count)
        # strawberry_[0-9], not strawberry_*: meshes/rejected/ holds one
        # that carries a base plate baked into the same triangle soup as
        # the plant, and the glob is what keeps it out of the greenhouse.
        glbs = sorted((ROOT / "meshes").glob("strawberry_[0-9].glb"))
        check("the textured strawberry variants are in the repository",
              len(glbs) >= 3, f"found {[m.name for m in glbs]}")
        if len(glbs) >= 3:
            for m in glbs:
                check(f"{m.name} carries its own texture",
                      glb_is_self_contained(m),
                      "a GLB that names a baseColorTexture and embeds no "
                      "image renders WHITE in Gazebo, silently")
                lo, hi = glb_bounds(m)
                check(f"{m.name} has a non-degenerate bounding box",
                      all(h > l for l, h in zip(lo, hi)),
                      f"{tuple(round(v, 3) for v in lo)} .. "
                      f"{tuple(round(v, 3) for v in hi)}")
                check(f"{m.name} fits the render budget",
                      triangle_count(m) < 15000,
                      f"{triangle_count(m):,} triangles")
                check(f"{m.name} reads the same through the dispatcher",
                      mesh_bounds(m) == (lo, hi),
                      "fit() goes through mesh_bounds, so a GLB that only "
                      "worked via glb_bounds would still be wrong")
            check("the variants are all DIFFERENT plants",
                  len({m.read_bytes() for m in glbs}) == len(glbs),
                  "duplicates were shipped once already, and 24 copies of "
                  "one asset read as 24 copies")

            gmeshed = Path(d) / "glb.sdf"
            build(default_source(), gmeshed)
            grep_ = build_meshes(gmeshed, glbs, 0.80, 1)
            gtext = gmeshed.read_text()
            check("every plant gets one of them", gtext.count("<mesh>") == 24,
                  grep_)
            check("and all of them are actually used",
                  all(gtext.count(m.name) > 0 for m in glbs),
                  {m.name: gtext.count(m.name) for m in glbs})
            # Scoped to the plant models: the greenhouse itself is full of
            # legitimate materials (walls, gutters, the 48 crosses).
            plant_blocks = re.findall(
                r'<model name="plant_\d+_\d+">.*?</model>', gtext, re.S)
            check("a TEXTURED mesh gets NO material override",
                  len(plant_blocks) == 24
                  and not any("<material>" in b for b in plant_blocks),
                  "overriding it repaints a photographed plant flat green, "
                  "which is the entire reason for using a GLB")
        if len(stls) >= 2 and len(glbs) >= 3:
            stl_blocks = re.findall(
                r'<model name="plant_\d+_\d+">.*?</model>', mtext, re.S)
            check("while an UNTEXTURED one still gets tinted",
                  all("<material>" in b for b in stl_blocks),
                  "otherwise the STL plants render bare white")

        # --- the procedural strawberry, for when there is no budget ------
        # It will not be mistaken for a bought asset. What it must get
        # right is the SILHOUETTE, and the silhouette is three facts:
        # a low rosette, trifoliate leaves, and fruit hanging BELOW the
        # canopy. The last is the one every stylised strawberry gets wrong.
        from agri.world.plants import (build_procedural,   # noqa: PLC0415
                                       procedural_plant)
        proc = Path(d) / "proc.sdf"
        build(default_source(), proc)
        rep = build_procedural(proc, 0.80, 1)
        ptext = proc.read_text()
        check("the procedural plants generate", "24 procedural" in rep, rep)
        try:
            xml.dom.minidom.parseString(ptext)
            check("and the world is still well-formed XML", True)
        except Exception as exc:                     # noqa: BLE001
            check("and the world is still well-formed XML", False, str(exc))
        check("the crosses survive that too",
              ptext.count('name="cross_') == 48)

        one = re.search(r'<model name="plant_1_1">.*?</model>', ptext,
                        re.S).group(0)
        parts = re.findall(r'<visual name="([a-z]+)\d', one)
        check("a plant has trifoliate leaves, three leaflets per stalk",
              parts.count("leaf") == 3 * parts.count("stalk"),
              f"{parts.count('leaf')} leaflets on {parts.count('stalk')} "
              "stalks -- three per stalk is the single most recognisable "
              "thing about the species")

        def zs(kind):
            return [float(m.split()[2]) for m in
                    re.findall(rf'<visual name="{kind}\d[^"]*"><pose>([^<]+)',
                               one)]
        check("the fruit hangs BELOW the leaf canopy",
              max(zs("berry")) < min(zs("leaf")),
              f"berries up to {max(zs('berry')):.3f}, leaves from "
              f"{min(zs('leaf')):.3f} -- a berry sitting on top of the "
              "leaves is the mistake every stylised strawberry makes")
        check("and the flowers are held above it",
              min(zs("flower")) > max(zs("leaf")) - 0.02)
        check("a plant carries more than one ripeness at once",
              len({re.search(r"<diffuse>([\d. ]+)", b).group(1)
                   for b in re.findall(
                       r'<visual name="berry\d">.*?</visual>', one, re.S)}) > 1,
              "a real plant is red, blush and green at the same time")
        check("two plants are not the same plant",
              procedural_plant("a", "0 0 0 0 0 0", 0.8, 1)
              != procedural_plant("b", "0 0 0 0 0 0", 0.8, 2))
        check("but one plant is the same on every run",
              procedural_plant("a", "0 0 0 0 0 0", 0.8, 7)
              == procedural_plant("a", "0 0 0 0 0 0", 0.8, 7),
              "regenerating twice must give an empty diff")
        check("collision is still a primitive, not 48 visuals",
              one.count('name="stem"') == 1 and "<collision" in one)
        refuses("and running it twice is refused",
                lambda: build_procedural(proc, 0.80, 1), ValueError)


def _ascii_stl(d: Path) -> Path:
    """A tiny ASCII STL, to prove the reader refuses one instead of
    mis-parsing it. Both formats share an extension, and a silent misread
    would place every plant somewhere plausible and wrong."""
    p = d / "ascii.stl"
    p.write_text("solid s\n facet normal 0 0 1\n  outer loop\n"
                 "   vertex 0 0 0\n   vertex 1 0 0\n   vertex 0 1 0\n"
                 "  endloop\n endfacet\nendsolid s\n")
    return p


def check_end_to_end() -> None:
    """The one that matters: request -> drive -> seal -> verify -> store."""
    print("\nend to end, through a loopback broker")
    from agri import keys
    from agri.cloud.store import Store
    from agri.cloud.server import Cloud
    from agri.protocol import TOPIC_REQUEST
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
            """A broker in a variable. Routes by topic prefix, synchronously,
            so a failure has a stack trace instead of a timeout."""

            def __init__(self):
                self.robot = None

            def publish(self, topic, payload, qos=1, retain=False):
                data = payload.encode() if isinstance(payload, str) else payload
                if topic.startswith("agri/v1/report/"):
                    self.last = cloud.on_report(data)
                elif topic.startswith("agri/v1/ack/"):
                    cloud.on_ack(data)
                elif topic.startswith("agri/v1/status/"):
                    cloud.on_status(data)

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

        # The replay. Byte-identical to the order it just obeyed, so every
        # cryptographic check passes -- it really is the Cloud's signature.
        check("and REFUSES the very same signed order a second time",
              link.on_request(json.dumps(signed).encode()) is None,
              "a signature proves who wrote an order, not that this is the "
              "first time you are reading it")

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

        # 9. the state now carries a nodes dict, not a single robot
        link.status(True, "idle")
        state = cloud.state()
        check("the state contains a nodes dict, not a single robot",
              "nodes" in state and isinstance(state["nodes"], dict)
              and "robot" not in state,
              str(list(state.keys())))
        check("the reporting robot appears in the nodes dict",
              "youbot-01" in state["nodes"],
              str(list(state["nodes"].keys())))


def check_telemetry() -> None:
    """Position and speed travel with every reading, and the Cloud has a
    console to order one from. Both were asked for explicitly."""
    print("\nwhat the robot transmits about ITSELF")
    import math                                        # noqa: PLC0415

    from agri.measurement import Measurement           # noqa: PLC0415
    from agri.protocol import make_status              # noqa: PLC0415

    m = Measurement(
        label="P2,5R",
        values={"temperature": 21.4, "humidity": 63.2, "luminosity": 12480,
                "co2": 431, "ph": 6.42},
        pose=(-3.1497, -1.7503, math.radians(0.23)),
        velocity=(0.004, -0.001, 0.0))

    qr = m.to_qr_text()
    check("the QR carries the position", "x=-3.150" in qr and "y=-1.750" in qr,
          qr)
    check("and the speed", "vx=0.004" in qr and "w=0.000" in qr, qr)
    check("the QR still fits comfortably", len(qr) < 200, f"{len(qr)} chars")
    # The Cloud re-reads the QR IMAGE and compares it with the numbers beside
    # it, so a rounding difference between the two would be reported as a
    # tampered report. This is the check that keeps them formatting alike.
    check("the QR survives a round trip through itself",
          Measurement.from_qr_text(qr).to_qr_text() == qr)
    check("and a round trip through the JSON, character for character",
          Measurement.from_dict(m.to_dict()).to_qr_text() == qr,
          "to_dict rounds; to_qr_text must round to the same places or the "
          "Cloud will reject its own robot's reports")

    d = m.to_dict()
    check("the JSON carries all three velocity components and the speed",
          set(d["velocity"]) == {"vx", "vy", "wz", "speed"}, str(d["velocity"]))
    check("the speed is the magnitude, not a component",
          abs(d["velocity"]["speed"] - math.hypot(0.004, 0.001)) < 1e-3)

    st = make_status("youbot-01", True, (-3.15, -1.75, 0.2), "driving",
                     velocity=(0.31, -0.02, 0.0))
    check("the live status heartbeat carries pose and speed",
          st["pose"]["x"] == -3.15 and st["velocity"]["speed"] == 0.311,
          str(st))
    check("the status carries a node_kind field",
          st["node_kind"] == "mobile", str(st))

    # The console is where the operator sits. Read as text: importing it is
    # fine, but what matters is that the verbs exist and that an unknown
    # line is treated as a request rather than an error.
    console = (ROOT / "agri" / "cloud" / "server.py").read_text()
    for verb in ("status", "coverage", "show", "csv", "help", "quit"):
        check(f"the Cloud console understands {verb!r}",
              f'"{verb}"' in console)
    check("and treats anything else as a set of targets",
          "self.request(line)" in console)
    check("the console owns the main thread, so orders can be typed live",
          "Console(cloud, publish).run()" in console)


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

    # The brake must not fire on the greenhouse itself. Both of these were
    # real: every leg to a headland stopped 0.27 m short of the end wall,
    # and a leg that shifted sideways while going forward tilted the
    # corridor onto the gutter it was driving beside.
    from agri.aisles import HEADLANDS, known_structure   # noqa: PLC0415
    from agri.catalogue import (SENSOR_OFFSET_X, WALL_X,  # noqa: PLC0415
                                WALL_Y)

    check("the end wall is recognised as the building",
          known_structure(-WALL_X, 0.0) and known_structure(WALL_X, 0.0))
    check("so are the side walls and the gutters",
          known_structure(0.0, WALL_Y) and known_structure(0.0, -1.0)
          and known_structure(-4.0, -1.2))
    check("clear floor is not",
          not known_structure(-2.0, -0.6) and not known_structure(-4.4, -1.85))
    check("and no station stands on what the brake would ignore",
          not any(known_structure(s.x, s.y) for s in st),
          "a station taken for structure is a station the robot may drive "
          "into something to reach")

    # The headland is the case that failed. Prove the wall really is inside
    # the brake's range there, so the check is about the fix and not about
    # an imaginary problem.
    base = HEADLANDS[0] - SENSOR_OFFSET_X
    c = brake_clearance(-WALL_X - base, 0.0, -1, 0)
    check("at the west headland the end wall IS within braking range",
          c is not None and -0.02 <= c < 0.35,
          f"clearance {c}; if this ever stops being true the fix below is "
          "guarding nothing")

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
    # The world is generated and no longer tracked, so a fresh clone has
    # none. Build it rather than skipping: the check below is about the
    # REAL generated world matching the bridge, and a skip here would pass
    # silently on exactly the machine most likely to be misconfigured.
    from agri.world import ensure as _ensure            # noqa: PLC0415
    _ensure.ensure()
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

    # An exception raised inside a paho callback unwinds paho's network
    # thread. The client then still PUBLISHES, from other threads, so the
    # broker and the dashboard both show a healthy robot -- while every
    # request is delivered to a client that is no longer reading. This
    # happened: status() asked for a pose before the first odometry arrived.
    check("the MQTT callbacks cannot let an exception escape",
          "def guarded(fn)" in node and "@guarded" in node,
          "one raise inside on_connect stops this node ever hearing the "
          "Cloud again, and nothing in the log says so")
    robot = (ROOT / "agri" / "robot.py").read_text()
    check("and status() never raises even with no odometry",
          "NEVER RAISES" in robot and "except Exception" in robot)

    # --- a broker that is not up yet is a WAIT, not a death --------------
    #
    # This is the whole of "my requests are not being executed". The node
    # called connect(), got ECONNREFUSED because mosquitto had not been
    # started yet, and main() returned 1. Gazebo stayed up, RViz drew the
    # robot, the dashboard looked healthy, and every order went nowhere --
    # with the only evidence eleven lines up in a log that had scrolled past.
    check("the robot node waits for the broker instead of dying",
          "connect_async(" in node and "self.client.connect(host" not in node,
          "one refused connection at startup used to kill the node, and the "
          "rest of the launch stayed up looking perfectly healthy")
    check("and main() no longer exits on it",
          "elif not node.connect()" not in node and "node.connect()" in node)
    check("and it retries with a bounded backoff",
          "reconnect_delay_set(" in node)
    check("it re-subscribes on EVERY connect, not just the first",
          "cl.subscribe(TOPIC_REQUEST" in node
          and "Re-subscribe on EVERY connect" in node,
          "a broker that restarted hands back an empty subscription table, "
          "and a robot connected to nothing looks exactly like one that works")
    check("it says so when the broker goes away",
          "on_disconnect" in node and "lost the broker" in node)
    check("and complains, on the wall clock, if none ever appears",
          "BROKER_GRACE_S" in node and "threading.Timer(BROKER_GRACE_S" in node,
          "use_sim_time is true here too: a ROS timer would be silent in "
          "exactly the half-started case this watchdog exists to report")
    check("and stops complaining once it is shutting down",
          "_quiet.set()" in node and "_quiet.is_set()" in node)
    check("the heartbeat does not publish into a dead client",
          "not self._connected" in node)

    # The 2D view. Without the TF nothing in RViz can be placed at all.
    viz = (pkg / "agri_robot" / "viz_node.py").read_text()
    rviz = (pkg / "config" / "agri.rviz").read_text()
    check("viz_node.py is valid Python", _parses(viz))
    check("something publishes map -> base_link",
          "TransformBroadcaster" in viz and '"base_link"' in viz,
          "RViz cannot draw a scan it cannot place")
    check("the RViz layout is fixed to the frame that node publishes",
          "Fixed Frame: map" in rviz and 'FRAME = "map"' in viz)
    check("it shows the lidar and the catalogue side by side",
          "/scan" in rviz and "/agri/stations" in rviz,
          "a scan on its own has no scale; the point is the comparison")
    # An empty RViz has five causes that look identical from the window, and
    # this node used to log NOTHING -- so "it is running but deaf" and "it
    # died" and "the view is wrong" were indistinguishable without a second
    # terminal and the presence of mind to use it while the launch was up.
    check("the viz node says it started",
          "agri_viz: publishing" in viz,
          "a node that logs nothing cannot be told apart from a dead one")
    check("and says when the odometry it needs first arrives",
          "first /odom" in viz)
    check("and complains, out loud, if it never does",
          "no /odom after" in viz and "ODOM_GRACE_S" in viz,
          "silence here is a blank RViz with no stated cause")
    # use_sim_time is true for this node, so a ROS timer only advances when
    # /clock does -- and /clock stopping is one of the faults the watchdog
    # exists to report. A watchdog driven by the thing it watches is silent
    # in precisely the case that matters.
    check("the no-odometry watchdog runs on the wall clock",
          "threading.Timer(delay, fn)" in viz
          and "ODOM_GRACE_S" in viz and "ODOM_ALARM_S" in viz,
          "a ROS timer here would never fire when the bridge is the fault")
    check("and cannot hold the process open", "t.daemon = True" in viz)
    # A watchdog that cannot be wrong OUT LOUD teaches the operator to
    # ignore it. This one fired at 12 s while Gazebo was still opening a
    # world with three textured meshes, said "check the gz bridge" about a
    # healthy bridge, and stayed the last word on the subject.
    check("it warns before it accuses",
          "_still_waiting" in viz and "still no /odom" in viz
          and viz.index("ODOM_GRACE_S = ") < viz.index("ODOM_ALARM_S = "))
    check("and the accusation waits a minute, not twelve seconds",
          "ODOM_ALARM_S = 60.0" in viz,
          "a laptop loading textured meshes takes longer than that, and "
          "the message was wrong every time it did")
    check("and it RETRACTS when the odometry finally arrives",
          "the warning above was a slow load" in viz and "_warned" in viz,
          "a warning nobody takes back is the log's last word on a fault "
          "that no longer exists")
    check("and it says how long the wait actually was",
          "took = time.monotonic() - self._started" in viz,
          "'it was slow' is a guess; 38 s is a number")

    live = ROOT / "tests" / "check_live.sh"
    check("there is a live diagnostic for the running system", live.exists())
    text = live.read_text()
    check("and it refuses to answer when the launch is not running",
          "agri_viz is not running" in text and "exit 1" in text,
          "measuring a system that was stopped a minute ago reports a fault "
          "that is real, useless, and indistinguishable from the true one")
    check("and it checks odometry before blaming the transform",
          text.index("/odom delivered nothing") < text.index("does NOT resolve"),
          "no /odom is the CAUSE of no map->base_link; reporting the "
          "symptom first sends you to the wrong file")

    # --- WHICH COPY of the RViz layout gets loaded -----------------------
    #
    # The file above is correct. It was correct for two debugging sessions,
    # during which RViz kept showing one display out of six -- because a
    # plain `colcon build` COPIES it into share/, the launch passed that copy
    # unconditionally, and the copy predated the fix. The world and the URDF
    # already preferred the source tree; this one asset did not, and that
    # asymmetry is what made the symptom unexplainable from the source.
    check("the RViz layout is resolved, not assumed",
          "_rviz_config(share)" in launch
          and 'arguments=["-d", rviz_cfg]' in launch,
          "passing share/config/agri.rviz directly means a stale install "
          "keeps serving a stale layout for ever")
    check("and the source tree wins over the installed copy",
          "RVIZ_IN_SOURCE" in launch and "_source_root()" in launch)
    check("and the launch says out loud which copy it handed over",
          "[view] RViz config:" in launch,
          "'RViz is showing something else' must never again be a thing you "
          "have to guess at")
    check("and warns when the installed copy has drifted",
          "DIFFERS and is" in launch and "read_bytes() != " in launch)

    check("the viz node is exposed as a console script",
          "viz_node = agri_robot.viz_node:main" in setup)
    check("and the launch file starts it",
          "executable=\"viz_node\"" in launch and '"rviz"' in launch)
    check("the viz node also gets a PYTHONPATH",
          launch.count('additional_env={"PYTHONPATH": py_path}') >= 2,
          "it imports agri.catalogue too, and dies on the system python "
          "exactly like the robot node did")

    # --- the RViz layout, which loaded ONE display out of five ----------
    #
    # Both mistakes below are silent: RViz drops what it cannot make sense
    # of and shows an empty Displays panel, which looks identical to a node
    # that is not publishing. That cost a debugging session, so the shape of
    # the file is asserted rather than trusted.
    vm = rviz.split("Visualization Manager:", 1)[-1].split("\nWindow Geometry")[0]
    keys = re.findall(r"^  ([A-Z][^:\n]*):", vm, re.M)
    check("the RViz config gives its display group an identity",
          'Class: ""' in vm and "Name: root" in vm,
          "RViz2 loads the tree by handing this node to DisplayGroup::load(); "
          "with no Class and no Name it loads no children and only the Grid "
          "survives (the Grid is built in)")
    check("and no key under Visualization Manager appears twice",
          len(keys) == len(set(keys)),
          f"yaml-cpp keeps the LAST of a repeated key, so the earlier one "
          f"and everything in it is discarded: {sorted(keys)}")
    displays = re.findall(r"^    - Class: (\S+)", rviz, re.M)
    check("every display names a real plugin class",
          displays and all(d.startswith("rviz_default_plugins/")
                           for d in displays),
          f"{displays}")
    check("the lidar display subscribes Best Effort",
          re.search(r"/scan\n\s+Reliability Policy: Best Effort", rviz),
          "the gz bridge publishes /scan Best Effort; a Reliable subscriber "
          "receives nothing and looks exactly like a broken lidar")
    # QoS has to MATCH, and a mismatch delivers nothing rather than warning.
    check("the marker display and the marker publisher agree on durability",
          ("TRANSIENT_LOCAL" in viz)
          == bool(re.search(r"/agri/stations\n\s+Durability Policy: "
                            r"Transient Local", rviz)),
          "Transient Local subscriber + Volatile publisher is INCOMPATIBLE "
          "in DDS: RViz would show nothing and say nothing")

    # --- the Gazebo camera ----------------------------------------------
    check("the launch file locks the Gazebo camera onto the robot",
          "/gui/track" in launch,
          "gz rewrites ~/.gz/sim/8/gui.config with the camera's last pose on "
          "every exit, so without this the robot spends the run off screen")
    # Compared as COMMANDS, not as text: both names also appear in the
    # comment that explains why, and the comment explains the deprecated one
    # first, which made an earlier version of this check fail on prose.
    check("it uses the track TOPIC and not only the deprecated service",
          launch.index("gz topic -t /gui/track")
          < launch.index("gz service -s /gui/follow"),
          "/gui/follow is deprecated in Harmonic: re-issuing it lets a "
          "watchdog detect the loss but never repair it")
    check("it keeps re-asserting rather than firing once",
          "while sleep" in launch and "seq 1 40" in launch,
          "the GUI answers 'data: true' before its render scene has the "
          "entity, and drops the track again minutes later")
    check("it has a fallback that cannot fail for want of an entity",
          "/gui/move_to/pose" in launch)
    check("and it follows the name the spawn actually gives the robot",
          re.search(r'"-name", "(\w+)"', launch).group(1) == "youbot"
          and 'name: "youbot"' in launch)
    check("the camera watchdog dies with the launch",
          "parent=$PPID" in launch,
          "bash in `sleep 20` ignores signals until the sleep returns, and a "
          "SIGKILLed launch never runs OnShutdown at all")


def check_two_machines() -> None:
    """The Cloud on one computer and the robot on another.

    This is the deployment the brief actually describes, and every check
    here is about the ONE step that has no error message of its own: the
    public keys have to be copied, and a side that quietly generates the
    missing one produces a system where nothing works and nothing complains.
    """
    print("\nthe Cloud on a second machine")
    from agri import keys                                # noqa: PLC0415

    with tempfile.TemporaryDirectory() as tmp:
        d = Path(tmp)
        # 1. One machine: an empty directory gets BOTH pairs, together.
        cloud_dir = d / "one"
        check("an empty key directory is bootstrapped with both pairs",
              keys.bootstrap(cloud_dir)
              and all(p.exists() for r in keys.ROLES
                      for p in keys.paths(r, cloud_dir)),
              "the single-machine case must stay a single command")
        check("and running again generates nothing",
              keys.bootstrap(cloud_dir) is False,
              "a second Cloud key makes every stored report undecryptable")

        # 2. Two machines, done RIGHT: private half stays, public is copied.
        robot_dir = d / "robot"
        robot_dir.mkdir()
        for name in ("robot_private.pem", "robot_public.pem",
                     "cloud_public.pem"):
            (robot_dir / name).write_bytes((cloud_dir / name).read_bytes())
        check("a robot machine with the copied public key starts",
              keys.public_of("cloud", robot_dir, generate=False)
              == keys.public_of("cloud", cloud_dir),
              "this is the whole deployment: copy the two public halves")
        check("and it does NOT hold the Cloud's private key",
              not (robot_dir / "cloud_private.pem").exists(),
              "capturing the robot must not give away the Cloud")

        # 3. Two machines, done WRONG: the copy was forgotten.
        forgot = d / "forgot"
        forgot.mkdir()
        for name in ("robot_private.pem", "robot_public.pem"):
            (forgot / name).write_bytes((cloud_dir / name).read_bytes())
        check("a populated directory is NOT bootstrapped behind your back",
              keys.bootstrap(forgot) is False,
              "generating a Cloud identity here is the trap: the robot would "
              "then refuse every real order as unsigned, silently")
        refuses("and the forgotten copy is refused, not invented",
                lambda: keys.public_of("cloud", forgot, generate=False),
                FileNotFoundError)
        try:
            keys.public_of("cloud", forgot, generate=False)
        except FileNotFoundError as exc:
            msg = str(exc)
        check("and the refusal says exactly what to copy where",
              "scp cloud_public.pem" in msg and "python3 -m agri.keys" in msg,
              msg)

    # Neither running program may take the generating path.
    server = (ROOT / "agri" / "cloud" / "server.py").read_text()
    node = (ROOT / "ros2" / "src" / "agri_robot" / "agri_robot"
            / "robot_node.py").read_text()
    # Matched as CALLS, not as text: the comment beside each one names
    # generate=False too, and counting the string counted the explanation.
    loads = re.compile(r"keys\.(?:ensure|public_of)\([^)]*\)")
    for who, src in (("Cloud", server), ("robot node", node)):
        calls = loads.findall(src)
        check(f"the {who} never generates the other side's key",
              len(calls) == 2 and all("generate=False" in c for c in calls)
              and "keys.bootstrap(" in src,
              f"one own keypair, one counterparty public key, neither "
              f"invented -- found {calls}")

    # The address the operator is told to open has to be the one that works.
    from agri.cloud.server import _is_loopback  # noqa: PLC0415
    check("the dashboard binds every interface by DEFAULT, not loopback",
          'ThreadingHTTPServer(' in server
          and '(args.http_host, args.http_port)' in server
          and '"--http-host", default=""' in server,
          "a Cloud nobody can reach from the robot's machine is not a Cloud")
    check("and an empty bind host is NOT mistaken for loopback",
          not _is_loopback("") and not _is_loopback("0.0.0.0"),
          "getting that backwards would make the no-token check permit "
          "exactly what it exists to forbid")
    check("while a real loopback address is recognised, by any spelling",
          all(_is_loopback(h) for h in ("127.0.0.1", "localhost", "::1",
                                        "127.0.0.5")))
    check("and prints an address that works from another machine",
          "_my_address()" in server and "192.0.2.1" in server,
          "printing localhost was true only while both ran on one computer")
    check("both sides take the broker address from the command line",
          "--broker" in server and '"broker"' in node,
          "MQTT is why this is portable: neither side needs the other's "
          "address, only the broker's")


def check_dashboard() -> None:
    """The web page, checked as source rather than as a screenshot.

    Two of these are re-checks of bugs the user found by looking at the
    page, which is the worst way to find them.
    """
    print("\nthe dashboard")
    from agri.catalogue import all_stations               # noqa: PLC0415

    d = ROOT / "agri" / "cloud" / "dashboard"
    js, html, css = ((d / n).read_text()
                     for n in ("app.js", "index.html", "style.css"))

    # --- mirrored text --------------------------------------------------
    # The map is drawn in world metres inside scale(1,-1) so that +y is up.
    # Glyphs inherit that flip and come out backwards; the row labels
    # shipped that way. Every text element must undo it.
    texts = re.findall(r"<text\b[^>]*>", js)
    bare = [t for t in texts if "upright(" not in t]
    check("every text on the map counter-flips the map's own transform",
          texts and not bare,
          f"these would render mirrored: {bare}")
    check("and it is written once, in a helper, not per call site",
          js.count("const upright =") == 1
          and js.count('transform="scale(1,-1)"') == 1,
          "one flip on the map's group, one helper that undoes it; a third "
          "copy is a second place to get the sign wrong")

    # --- overlapping stations -------------------------------------------
    # P1,jL and P2,jR share an inner aisle 0.10 m apart. Their value chips
    # are 0.22 m tall, so drawn on the same side one hides the other
    # exactly. The fix pushes each chip AWAY from its own plant row.
    off = float(re.search(r'const away = .*?\* ([\d.]+);', js).group(1))
    high = float(re.search(r'height="([\d.]+)" rx=', js).group(1))
    same_x = {}
    for s in all_stations():
        same_x.setdefault(round(s.x, 6), []).append(s)
    marks = min(abs(a.y - b.y)
                for g in same_x.values() for a in g for b in g if a is not b)
    chips = min(abs((a.y + (-off if a.side == "R" else off))
                    - (b.y + (-off if b.side == "R" else off)))
                for g in same_x.values() for a in g for b in g if a is not b)
    check("the closest two stations really do collide when drawn plainly",
          marks < high,
          f"{marks:.2f} m apart, chip {high:.2f} m tall -- if this ever "
          "stops being true the offset below is no longer needed")
    check("pushing each chip away from its own row separates them",
          chips > high + 0.05,
          f"gap between chip centres is {chips:.2f} m, chip is {high:.2f} m")
    check("a stem says which mark each chip belongs to",
          'class="stem"' in js and ".stem {" in css,
          "an offset label with nothing tying it to its mark is worse than "
          "an overlapping one")
    check("the drawn cross is no wider than the gap between two stations",
          2 * float(re.search(r"const a = ([\d.]+);", js).group(1)) <= marks,
          "arms longer than the spacing make one X out of two crosses")

    # --- the filters the operator asked for ------------------------------
    check("the map can be narrowed to one row, one side, or one station",
          all(f'id="{i}"' in html for i in ("f-row", "f-side", "f-jump")),
          "no drawing separates marks 0.10 m apart at this scale; the map "
          "has to be able to show fewer of them")
    check("the filters are wired up",
          all(f'getElementById("{i}")' in js
              for i in ("f-row", "f-side", "f-jump")))
    check("and the open station is never filtered out from under its panel",
          "|| s.label === S.selected" in js,
          "the detail panel would describe a mark that is not on the map")

    # Generic, and the reason the three above cannot rot into false passes:
    # every id app.js reaches for must exist in the page.
    wanted = sorted(set(re.findall(r'getElementById\("([^"]+)"\)', js)))
    missing = [i for i in wanted if f'id="{i}"' not in html]
    check("every element app.js addresses exists in index.html",
          not missing, f"missing: {missing}")

    # The theme, which the operator said was too dark to read.
    light = css.split("@media", 1)[0]
    check("the default theme is the light one",
          "--bg:" in light and "prefers-color-scheme: dark" in css,
          ":root must define the LIGHT palette, with dark as the override")

    # --- the two media panes are the same size ---------------------------
    # The QR code is square and the photograph is 4:3, so left alone they
    # rendered at different heights and the pair looked misaligned.
    check("the photograph and the QR code render at the same size",
          "--media-size" in css and "object-fit: contain" in css,
          "both panes must be one fixed box, with the photo fitted inside")
    check("and the photograph is letterboxed, not cropped",
          "object-fit: cover" not in css,
          "cropping a plant photograph to tidy a layout throws away the "
          "evidence the photograph exists to carry")

    # --- the measurement table page --------------------------------------
    tbl = (d / "table.html").read_text()
    tjs = (d / "table.js").read_text()
    check("there is a table page", (d / "table.html").exists())
    check("and the Cloud serves it at /table",
          '"/table"' in (ROOT / "agri" / "cloud" / "server.py").read_text())
    check("every element table.js addresses exists in table.html",
          not [i for i in re.findall(r'getElementById\("([^"]+)"\)', tjs)
               if f'id="{i}"' not in tbl],
          str([i for i in re.findall(r'getElementById\("([^"]+)"\)', tjs)
               if f'id="{i}"' not in tbl]))
    check("the table is one row per PLANT, not one per station",
          "R${" not in tjs and "sideCells(R) + sideCells(L)" in tjs,
          "the comparison that carries the information is R against L on "
          "the same plant; one station per row hides it")
    check("the right-hand station comes first, then the left",
          tjs.index("sideCells(R)") < tjs.index("sideCells(L)"))
    check("both sides carry a link to their photograph and their QR",
          tjs.count('media(s.photo') == 1 and tjs.count('media(s.qr') == 1)
    check("media are links rather than 96 inline images",
          "<a href=" in tjs and "<img" not in tjs,
          "ninety-six images inline would make the page unusable on the "
          "laptop it is demonstrated from")
    check("a station never measured shows a dash, not a zero",
          '"—"' in tjs and "not measured" in tjs)
    check("the plant column stays put when the table scrolls sideways",
          "position: sticky" in css and "th.stick" in css,
          "sixteen data columns do not fit; scrolling right must not take "
          "away the only thing saying which row you are reading")
    check("the two pages link to each other",
          'href="/table"' in (d / "index.html").read_text()
          and 'href="/"' in tbl)
    check("and the token is carried across both links",
          "/table?token=" in (d / "app.js").read_text()
          and "/?token=" in tjs,
          "otherwise the page renders and every fetch behind it 401s, "
          "which looks like a broken table rather than a missing credential")

    # --- plants.csv is the same shape as the page ------------------------
    from agri.cloud.store import Store                        # noqa: PLC0415
    st = Store(Path(tempfile.mkdtemp()) / "s")
    cols = st.PLANT_COLUMNS
    check("plants.csv is one row per plant, both sides on the line",
          cols[:3] == ["plant", "row", "index"]
          and any(c.startswith("R_") for c in cols)
          and any(c.startswith("L_") for c in cols))
    check("and its right block really precedes its left block",
          max(i for i, c in enumerate(cols) if c.startswith("R_"))
          < min(i for i, c in enumerate(cols) if c.startswith("L_")),
          "the file and the page must be readable the same way")
    check("and each side carries its own photo and QR path",
          {"R_photo", "R_qr", "L_photo", "L_qr"} <= set(cols))
    rows = list(csv.DictReader(
        st.export_plants_csv(Path(tempfile.mkdtemp()) / "p.csv")
        .read_text().splitlines()))
    check("an empty store still writes all 24 plants", len(rows) == 24,
          f"{len(rows)} rows")
    check("and an unmeasured side is EMPTY, not zero",
          rows[0]["R_temperature"] == "" and rows[0]["L_photo"] == "",
          f"got {rows[0]['R_temperature']!r}")
    srv = (ROOT / "agri" / "cloud" / "server.py").read_text()
    check("every export path writes the per-plant file too",
          srv.count("export_plants_csv") == 3,
          f"{srv.count('export_plants_csv')} of 3 call sites -- the console, "
          "the dashboard button and the shutdown must agree")


def check_dashboard_auth() -> None:
    """The token, over a real socket, because the leak is in the URL."""
    print("\nthe dashboard's front door")
    import urllib.error                                   # noqa: PLC0415
    import urllib.request                                 # noqa: PLC0415
    from functools import partial                         # noqa: PLC0415
    from http.server import ThreadingHTTPServer           # noqa: PLC0415

    from agri import keys                                 # noqa: PLC0415
    from agri.cloud.server import (COOKIE, Cloud, Handler,    # noqa: PLC0415
                                   _is_loopback, _Throttle,
                                   no_auth_refusal)
    from agri.cloud.server import main as server_main     # noqa: PLC0415
    from agri.cloud.store import Store                    # noqa: PLC0415

    class NoRedirect(urllib.request.HTTPRedirectHandler):
        """Follow nothing: the redirect IS what is being tested."""

        def redirect_request(self, *a, **kw):
            return None

    opener = urllib.request.build_opener(NoRedirect)

    def fetch(url, cookie=""):
        req = urllib.request.Request(url)
        if cookie:
            req.add_header("Cookie", cookie)
        try:
            with opener.open(req, timeout=5) as r:
                return r.status, dict(r.headers), r.read()
        except urllib.error.HTTPError as e:
            return e.code, dict(e.headers), e.read()

    with tempfile.TemporaryDirectory() as d:
        d = Path(d)
        keys.bootstrap(d / "keys")
        cloud = Cloud(Store(d / "store"), d / "keys")
        token = "s3cret-token"
        httpd = ThreadingHTTPServer(
            ("127.0.0.1", 0),
            # Its OWN throttle. A lockout is per server, not per process:
            # this test deliberately triggers one, and the next test serves
            # from the same 127.0.0.1.
            partial(Handler, cloud, lambda *a: None, token, None,
                    throttle=_Throttle()))
        base = f"http://127.0.0.1:{httpd.server_address[1]}"
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        try:
            # 1. The naked token in the URL is accepted ONCE, and the very
            #    first thing it buys is being got rid of.
            code, hdr, _ = fetch(f"{base}/?token={token}")
            check("opening the dashboard with ?token= redirects",
                  code == 303, f"{code}")
            check("to the SAME page with no token in the address",
                  hdr.get("Location") == "/", hdr.get("Location"))
            cookie = hdr.get("Set-Cookie", "")
            check("and hands over a cookie instead",
                  cookie.startswith(f"{COOKIE}={token}"), cookie)
            check("which no script can read",
                  "HttpOnly" in cookie,
                  "a cookie JavaScript cannot read is a cookie an injected "
                  "script cannot steal")
            check("and which no other site can send",
                  "SameSite=Strict" in cookie,
                  "without it, a page in another tab could POST /api/request "
                  "and drive the robot")
            check("and no page ever leaks its own URL onward",
                  hdr.get("Referrer-Policy") == "no-referrer",
                  "the token was in that URL a moment ago")

            # 2. From then on the cookie alone works, with nothing in the URL.
            jar = f"{COOKIE}={token}"
            code, _, body = fetch(f"{base}/api/state", cookie=jar)
            check("the cookie alone authorises the API", code == 200, str(code))
            check("and the answer is the real state, not a stub",
                  b"summary" in body, body[:80])
            code, _, _ = fetch(f"{base}/api/state")
            check("while no credential at all is refused", code == 401)
            code, _, _ = fetch(f"{base}/api/state", cookie=f"{COOKIE}=wrong")
            check("and a wrong cookie is refused too", code == 401)

            # 3. The table page carries the same cookie, so the link between
            #    the two pages no longer has to smuggle the token.
            code, hdr, _ = fetch(f"{base}/table?token={token}")
            check("the table page sheds its token the same way",
                  code == 303 and hdr.get("Location") == "/table",
                  f"{code} {hdr.get('Location')}")

            # 4. An API call that arrives with a naked token still works --
            #    curl, a script, a browser with cookies off -- and is handed
            #    the cookie so the next one need not carry it.
            code, hdr, _ = fetch(f"{base}/api/state?token={token}")
            check("a token on an API call is still accepted", code == 200)

            # 5. Guessing. A 16-byte token is not brute-forced in ten tries;
            #    the point is that a script cannot try a million overnight.
            codes = [fetch(f"{base}/api/state?token=wrong{i}")[0]
                     for i in range(14)]
            check("repeated wrong tokens start being refused outright",
                  429 in codes,
                  f"got {codes} -- an unattended dashboard on a laptop is "
                  "exactly what a slow guessing script is for")
            check("and the refusal says how long to wait",
                  fetch(f"{base}/api/state?token=nope")[1].get("Retry-After"),
                  "a 429 with no Retry-After is a client that hammers on")
            # The right token must still work once the lockout expires, and
            # a locked-out attacker must not lock out the operator forever:
            # this asserts the lockout is bounded, not that it is absent.
            from agri.cloud.server import AUTH_LOCKOUT_S   # noqa: PLC0415
            check("the lockout is measured in a minute, not forever",
                  0 < AUTH_LOCKOUT_S <= 300, f"{AUTH_LOCKOUT_S} s")
        finally:
            httpd.shutdown()

    # 6. The browser has to actually SEND the cookie, or every page would
    #    quietly fall back to the URL token and none of the above would
    #    have changed anything.
    dash = ROOT / "agri" / "cloud" / "dashboard"
    for name in ("app.js", "table.js"):
        src = (dash / name).read_text()
        # Up to the end of the call, not the end of the line: the POST
        # that issues a request spans four lines, and a line-based match
        # would have declared it fine while it sent no cookie at all.
        fetches = [src[i:src.index(");", i) + 2]
                   for i in _find_all(src, "fetch(apiUrl(")]
        check(f"{name} sends credentials on every API call",
              bool(fetches) and all("CRED" in f for f in fetches),
              f"these would not carry the cookie: "
              f"{[f for f in fetches if 'CRED' not in f]}")
        check(f"{name} asks for same-origin credentials, not 'include'",
              'credentials: "same-origin"' in src,
              "'include' would send the cookie cross-origin as well")

    # 7. The open door. --dashboard-token '' was a flag with a warning
    #    printed under it, and a startup warning scrolls away.
    check("an empty bind address is not loopback", not _is_loopback(""))
    rc = server_main(["--dashboard-token", "", "--http-port", "0"])
    check("no token on a public interface REFUSES to start", rc == 2,
          f"exit {rc} -- it used to print a warning and serve anyway")
    msg = no_auth_refusal("", 8088)
    check("and the refusal explains both ways out",
          "--http-host 127.0.0.1" in msg and "drop --dashboard-token" in msg,
          msg)


def check_mode_buttons() -> None:
    """The mode selector, over a real socket, because a console verb is
    invisible to somebody watching a screen."""
    print("\nthe dashboard's mode selector")
    import urllib.error                                   # noqa: PLC0415
    import urllib.request                                 # noqa: PLC0415
    from functools import partial                         # noqa: PLC0415
    from http.server import ThreadingHTTPServer           # noqa: PLC0415

    from agri import keys                                 # noqa: PLC0415
    from agri.cloud.server import Cloud, Handler          # noqa: PLC0415
    from agri.cloud.store import Store                    # noqa: PLC0415
    from agri.protocol import MODE_COLLECTOR, MODE_COMMAND  # noqa: PLC0415

    def post(url, payload):
        req = urllib.request.Request(
            url, data=json.dumps(payload).encode(), method="POST",
            headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read() or b"{}")
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read() or b"{}")

    with tempfile.TemporaryDirectory() as d:
        d = Path(d)
        keys.bootstrap(d / "keys")
        cloud = Cloud(Store(d / "store"), d / "keys")
        token = "s3cret-token"
        httpd = ThreadingHTTPServer(
            ("127.0.0.1", 0),
            partial(Handler, cloud, lambda *a: None, token, None))
        base = f"http://127.0.0.1:{httpd.server_address[1]}"
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        try:
            check("the Cloud starts in command mode",
                  cloud.mode == MODE_COMMAND and cloud.receiving)

            code, body = post(f"{base}/api/mode?token={token}",
                              {"mode": "collector"})
            check("the dashboard can switch to collector",
                  code == 200 and cloud.mode == MODE_COLLECTOR,
                  f"{code} {body}")
            check("and the answer says what it now is, so the page never "
                  "has to guess", body.get("mode") == MODE_COLLECTOR
                  and body.get("receiving") is True, str(body))
            check("and the switch is recorded in the dialogue the operator "
                  "is reading",
                  any("collector mode" in e["text"] for e in cloud.dialogue),
                  str([e["text"] for e in cloud.dialogue][-3:]))

            code, body = post(f"{base}/api/mode?token={token}",
                              {"receiving": False})
            check("the Cloud can be told to stop receiving",
                  code == 200 and cloud.receiving is False, f"{code} {body}")
            code, body = post(f"{base}/api/mode?token={token}",
                              {"receiving": True})
            check("and to start again", cloud.receiving is True)

            code, body = post(f"{base}/api/mode?token={token}",
                              {"mode": "nonsense"})
            check("a mode that does not exist is REFUSED, not ignored",
                  code == 400 and cloud.mode == MODE_COLLECTOR,
                  f"{code} {body} -- a button that silently does nothing is "
                  "worse than one that errors")
            check("and the refusal names the two that do exist",
                  "command" in str(body) and "collector" in str(body),
                  str(body))

            code, _ = post(f"{base}/api/mode", {"mode": "command"})
            check("and the endpoint is behind the token like every other",
                  code == 401 and cloud.mode == MODE_COLLECTOR)

            # The mode has to reach the page, or the buttons cannot show
            # which one is in force.
            st = cloud.state()
            check("the state carries the mode, the pause and the dialogue",
                  st["mode"] == MODE_COLLECTOR and "receiving" in st
                  and isinstance(st.get("dialogue"), list))
        finally:
            httpd.shutdown()

    # And the page has to actually use it.
    js = (ROOT / "agri" / "cloud" / "dashboard" / "app.js").read_text()
    html = (ROOT / "agri" / "cloud" / "dashboard" / "index.html").read_text()
    css = (ROOT / "agri" / "cloud" / "dashboard" / "style.css").read_text()
    check("there are two mode buttons, named in the operator's language",
          'data-mode="command"' in html and 'data-mode="collector"' in html
          and "COLLECTEUR" in html)
    check("and they are wired to the endpoint",
          'setMode({ mode: b.dataset.mode })' in js
          and '"/api/mode"' in js)
    check("the one in force is visibly the one in force",
          'classList.toggle("on"' in js and ".seg button.on" in css,
          "a selector that does not show its state is a selector nobody "
          "trusts")
    check("the pause button is DISABLED in command mode, not silently inert",
          'btn.disabled = mode !== "collector"' in js,
          "in command mode the robot was asked for the reading and sends "
          "it; a pause there would do nothing and explain nothing")
    check("and each mode says what it does, on screen",
          "MODE_WHY" in js and "negotiates" in js,
          "the two modes are the demonstration; naming them is not enough")
    check("the handshake is shown beside the buttons that cause it",
          'id="dialogue"' in html and ".dialogue .cloud .who" in css,
          "the whole point of collector mode is the conversation")
    check("and the two directions are told apart without reading the words",
          "CLOUD ──►" in js and "◄── NODE" in js,
          "the same arrows the terminal log uses")


def check_session_buttons() -> None:
    """ENREGISTRER and QUITTER, exercised over a real socket.

    String-matching the JavaScript would pass just as happily against a
    button wired to an endpoint that answers 404, which is exactly the bug
    worth catching: the operator presses it once, at the end of a run, and
    finds out then whether the CSV exists.
    """
    print("\nthe dashboard's session buttons")
    import time                                           # noqa: PLC0415
    import urllib.error                                   # noqa: PLC0415
    import urllib.request                                 # noqa: PLC0415
    from functools import partial                         # noqa: PLC0415
    from http.server import ThreadingHTTPServer           # noqa: PLC0415

    from agri import keys                                 # noqa: PLC0415
    from agri.cloud.server import Cloud, Handler          # noqa: PLC0415
    from agri.cloud.store import Store                    # noqa: PLC0415

    def post(url):
        req = urllib.request.Request(url, data=b"", method="POST")
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read() or b"{}")
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read() or b"{}")

    def get(url):
        try:
            with urllib.request.urlopen(url, timeout=5) as r:
                return r.status, r.read()
        except urllib.error.HTTPError as e:
            return e.code, e.read()

    with tempfile.TemporaryDirectory() as d:
        d = Path(d)
        keys.bootstrap(d / "keys")
        cloud = Cloud(Store(d / "store"), d / "keys")
        stopped = []
        token = "s3cret-token"
        httpd = ThreadingHTTPServer(
            ("127.0.0.1", 0),
            partial(Handler, cloud, lambda *a: None, token,
                    lambda: stopped.append(True)))
        base = f"http://127.0.0.1:{httpd.server_address[1]}"
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        try:
            code, body = post(f"{base}/api/export?token={token}")
            check("ENREGISTRER exports the CSV", code == 200, f"{code} {body}")
            check("and says where it went, on disk and over HTTP",
                  body.get("path", "").endswith("measurements.csv")
                  and body.get("url") == "/media/measurements.csv",
                  str(body))
            check("and the file it names really is there",
                  Path(body["path"]).exists(), body.get("path"))

            code, raw = get(f"{base}{body['url']}?token={token}")
            check("and the browser can download it from that URL",
                  code == 200 and raw.startswith(b"label,"), f"{code} {raw[:40]}")

            # The button now downloads TWO files. A second URL that answers
            # 404 would look identical to the first one working, because the
            # browser reports nothing for a failed synthetic download.
            check("ENREGISTRER exports the requests log as well",
                  body.get("requests_url") == "/media/requests.csv",
                  str(body))
            code, raw = get(f"{base}{body['requests_url']}?token={token}")
            check("and that one downloads too",
                  code == 200 and raw.startswith(b"request_id,"),
                  f"{code} {raw[:40]}")

            # Measurements are measurements whether they arrive as JSON or as
            # a file; a token that guards one and not the other guards nothing.
            code, _ = get(f"{base}/media/measurements.csv")
            check("media is behind the token too, not just /api",
                  code == 401, f"got {code}, expected 401")
            code, _ = post(f"{base}/api/export")
            check("and so is the export itself", code == 401,
                  f"got {code}, expected 401")

            code, body = post(f"{base}/api/quit?token={token}")
            check("QUITTER answers before it stops", code == 200, f"{code} {body}")
            # WAIT for it, do not read it. _quit() replies and flushes
            # BEFORE calling the shutdown hook -- deliberately, so the
            # browser gets its answer rather than a connection reset -- so
            # the client can be back here while the handler thread has not
            # reached the hook yet. Sampling the flag the instant the POST
            # returns is a race the test loses on a loaded machine and wins
            # on an idle one, which is the worst kind of check: it passed
            # here and failed on the operator's laptop.
            for _ in range(100):
                if stopped:
                    break
                time.sleep(0.02)
            check("and actually triggers the shutdown", stopped == [True],
                  "the button replied 200 and did not stop the Cloud "
                  "within 2 s")
        finally:
            httpd.shutdown()

    # A Cloud nobody owns the lifetime of must refuse, not kill its caller.
    with tempfile.TemporaryDirectory() as d:
        d = Path(d)
        keys.bootstrap(d / "keys")
        cloud = Cloud(Store(d / "store"), d / "keys")
        httpd = ThreadingHTTPServer(
            ("127.0.0.1", 0),
            partial(Handler, cloud, lambda *a: None, "", None))
        base = f"http://127.0.0.1:{httpd.server_address[1]}"
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        try:
            code, body = post(f"{base}/api/quit")
            check("a Cloud with no shutdown hook refuses instead of dying",
                  code == 403, f"{code} {body}")
        finally:
            httpd.shutdown()

    demo = (ROOT / "agri" / "demo.py").read_text()
    check("and the offline demo is one of those",
          'partial(Handler, cloud, publish, "", None)' in demo,
          "the demo's dashboard must not be able to kill its own process")


def check_route() -> None:
    """The order the stations are driven in, and what it costs.

    The robot used to drive the survey order it was handed. That order pairs
    each plant's two sides -- P1,1R then P1,1L -- and those two sides have an
    8 m gutter between them, so every pair meant a trip to a headland and
    back. Measured over 48 stations it was 87 % of the distance.

    The reordering can only be trusted if it cannot make things worse, so
    that is what most of this section checks: the set is preserved exactly,
    and the chosen order is never longer than the one issued -- over random
    subsets, not just the full sweep.
    """
    print("\nthe route, and the battery it costs")
    from agri.catalogue import SENSOR_OFFSET_X
    from agri.cloud.server import Cloud
    from agri.cloud.store import Store
    from agri.labels import all_labels
    from agri.protocol import make_ack
    from agri.survey import (BATTERY_WH, CRUISE_MPS, crossings, energy_wh,
                             leg_lengths, path_length, survey_order)

    labels = all_labels()
    # Where the launch file spawns the base, as the SENSOR point -- the frame
    # every station is expressed in.
    sx, sy = -4.58 + SENSOR_OFFSET_X, -1.85

    issued_m = path_length(labels, sx, sy)
    order = survey_order(labels, sx, sy)
    best_m = path_length(order, sx, sy)

    check("a route's length is measured along the legs the driver will fly",
          abs(sum(leg_lengths(labels[:3], sx, sy))
              - path_length(labels[:3], sx, sy)) < 1e-9)
    check("the survey order really is as expensive as it looked",
          issued_m > 250, f"{issued_m:.1f} m over 48 stations")
    check("and the reordered one is a fraction of it",
          best_m < issued_m / 5,
          f"{issued_m:.0f} m -> {best_m:.0f} m")
    print(f"        {issued_m:.0f} m -> {best_m:.0f} m "
          f"({100 * (issued_m - best_m) / issued_m:.0f} % less), "
          f"{crossings(labels, sx, sy)} -> {crossings(order, sx, sy)} "
          f"gutter crossings")

    check("the mission still visits every station it was given",
          sorted(order) == sorted(labels),
          "reordering must never drop or invent a station")
    check("and visits none of them twice",
          len(order) == len(set(order)))
    check("the saving comes from not crossing the gutters",
          crossings(order, sx, sy) <= 3,
          f"{crossings(order, sx, sy)} crossings; there are four bands, so "
          "three is the fewest a full sweep can do")

    # THE PROPERTY THAT MAKES THIS SAFE TO TURN ON. The issued order is one
    # of the candidates and wins ties, so adopting a longer route is not a
    # bug that could happen -- it is a thing the function cannot do.
    rng = random.Random(20260808)
    worse = same = 0
    for n in (2, 3, 4, 7, 16, 33):
        for _ in range(120):
            sub = rng.sample(labels, n)
            got = survey_order(sub, sx, sy)
            if sorted(got) != sorted(sub):
                worse = -1
                break
            a, b = path_length(sub, sx, sy), path_length(got, sx, sy)
            if b > a + 1e-9:
                worse += 1
            if got == sub:
                same += 1
    check("no subset of stations is ever made longer by reordering",
          worse == 0, f"{worse} of 720 random subsets got worse")
    check("and an order that saves nothing is left alone",
          same > 0, "the issued order must win ties, or a two-station "
                    "request gets shuffled for no reason")

    check("a request too small to reorder is returned untouched",
          survey_order(["P2,4R", "P2,4L"], sx, sy) == ["P2,4R", "P2,4L"])
    check("and an unknown label does not take the mission down with it",
          survey_order(["P1,1R", "NOPE", "P3,8L"], sx, sy)
          == ["P1,1R", "NOPE", "P3,8L"],
          "the driver refuses it and says which; the planner must not raise")
    check("the order is stable: the same request plans the same way twice",
          survey_order(labels, sx, sy) == survey_order(labels, sx, sy))

    # --- the mission carries both orders -------------------------------
    from agri.robot import Mission
    m = Mission("r", targets=list(labels))
    check("a fresh mission has not reordered anything yet", not m.reordered)
    check("and it kept the order the Cloud issued", m.issued == labels)
    m.targets = order
    check("once planned it knows it reordered", m.reordered)
    check("and the issued order is still there to compare against",
          m.issued == labels,
          "an operator reads what they asked for, not what was optimal")

    src = (ROOT / "agri" / "robot.py").read_text()
    check("the plan is made where the robot's position is known",
          "def plan(" in src and "self.visitor.driver.pose()" in src,
          "the best order depends on where the robot is standing")
    check("and run_mission plans before it drives",
          src.index("self.plan()") < src.index("for label in m.targets"))
    check("no odometry is a reason to drive the issued order, not to refuse",
          "keeping the issued order" in src)
    check("every ack still names its own station",
          'self.ack(m.request_id, "driving", label)' in src,
          "this is what makes reordering invisible to anyone reading acks")

    # --- distance is measured, energy is modelled ----------------------
    drv = (ROOT / "ros2" / "src" / "agri_robot" / "agri_robot"
           / "driver.py").read_text()
    check("the driver counts the metres it actually drove",
          "def distance_m(" in drv)
    check("from successive positions, not by integrating the speed",
          "self._driven += step" in drv)
    check("and a teleport is not counted as distance driven",
          "ODOM_JUMP_M" in drv and "step < ODOM_JUMP_M" in drv,
          "a respawn would otherwise add metres nothing spent")

    check("the closing ack carries the distance",
          "metres" in make_ack("r", "b", "finished", metres=41.2))
    check("and what the plan expected, beside it",
          make_ack("r", "b", "finished", planned_m=40.0)["planned_m"] == 40.0)
    check("a robot that does not count sends neither, rather than zero",
          "metres" not in make_ack("r", "b", "finished"),
          "zero metres driven and 'this robot does not say' are different "
          "facts and must not export as the same cell")

    e = energy_wh(metres=100.0, seconds=1000.0)
    check("energy is split into idle and drive, not given as one number",
          {"idle_wh", "drive_wh", "total_wh"} <= set(e))
    check("and the two parts add up to the total",
          abs(e["idle_wh"] + e["drive_wh"] - e["total_wh"]) < 0.02)
    check("driving longer than the mission lasted is impossible",
          energy_wh(1e6, 10.0)["driving_s"] <= 10.0,
          "a distance that cannot fit in the time would otherwise invent "
          "drive energy out of a bad odometry reading")
    check("a mission that drove nothing still costs idle power",
          energy_wh(0.0, 600.0)["total_wh"] > 0,
          "this is the finding: shortening the route attacks the smaller "
          "half of the bill")
    check("the battery is expressed as a percentage of a stated pack",
          0 < energy_wh(41.0, 900.0)["battery_pct"] < 100 and BATTERY_WH > 0)
    check("cruise speed matches the driver's, or the estimate is fiction",
          abs(CRUISE_MPS - float(re.search(r"^V_MAX\s*=\s*([\d.]+)", drv,
                                           re.M).group(1))) < 1e-9,
          "agri.survey.CRUISE_MPS and driver.V_MAX are the same speed")

    surv = (ROOT / "agri" / "survey.py").read_text()
    check("the energy constants are labelled as assumptions, out loud",
          "ASSUMPTIONS, NOT MEASUREMENTS" in surv,
          "a modelled number sitting beside forty-eight measured ones has "
          "to say which it is")

    # --- it reaches the CSV and the dashboard ---------------------------
    cloud = Cloud(Store(Path(tempfile.mkdtemp()) / "s"),
                  Path(tempfile.mkdtemp()) / "k")
    rid, _ = cloud.build_request("ALL")
    check("a request in flight has no distance yet",
          cloud.progress[rid]["metres"] is None)
    cloud.on_ack(json.dumps(make_ack(rid, "youbot-01", "finished", done=48,
                                     total=48, metres=41.2,
                                     planned_m=40.1)).encode())
    p = cloud.progress[rid]
    check("the Cloud records the distance the robot reported", p["metres"] == 41.2)
    check("and what the robot planned", p["planned_m"] == 40.1)

    rows = list(csv.DictReader(
        cloud.export_requests_csv(Path(tempfile.mkdtemp()) / "r.csv")
        .read_text().splitlines()))
    check("requests.csv names its driven_m column",
          "driven_m" in rows[0])
    check("and its planned_m column", "planned_m" in rows[0])
    check("and marks the energy columns as estimates",
          all(c in rows[0] for c in ("energy_wh_est", "idle_wh_est",
                                     "drive_wh_est")),
          "the header is where a reader finds out which numbers are modelled")
    check("and the row really carries the distance",
          rows[0]["driven_m"] == "41.2", rows[0])

    cloud2 = Cloud(Store(Path(tempfile.mkdtemp()) / "s"),
                   Path(tempfile.mkdtemp()) / "k")
    rid2, _ = cloud2.build_request("P2,4R")
    blank = list(csv.DictReader(
        cloud2.export_requests_csv(Path(tempfile.mkdtemp()) / "r.csv")
        .read_text().splitlines()))[0]
    check("a request with no distance leaves the cells EMPTY, not zero",
          blank["driven_m"] == "" and blank["energy_wh_est"] == "",
          f"got {blank['driven_m']!r} and {blank['energy_wh_est']!r}")

    st = cloud.state()["requests"][-1]
    check("the dashboard is given the energy already worked out",
          st.get("energy_wh") is not None)
    app = (ROOT / "agri" / "cloud" / "dashboard" / "app.js").read_text()
    check("the dashboard shows the distance driven",
          "drove ${r.metres.toFixed(1)} m" in app)
    check("and says the energy is an estimate where it shows it",
          "Wh est" in app)
    check("and shows nothing at all when the robot did not report one",
          "r.metres == null" in app,
          "a robot that does not count must not read as one that drove 0.0 m")


def check_modes() -> None:
    """Two modes, one protocol, and an exchange you can read.

    Collector mode is the one worth checking hard, because everything it
    adds is a NEGOTIATION: the Cloud asks before ordering, the robot asks
    before sending, and either side may say no. Each of those refusals is a
    path that only runs when something is wrong, which is exactly the kind
    of path that rots unnoticed.
    """
    print("\nthe two modes: command and collector")
    import subprocess
    import threading as _th
    import time

    from agri import keys as _keys
    from agri.cloud.server import Cloud
    from agri.cloud.store import Store
    from agri.crypto_ecc import verify_json
    from agri.protocol import (MODE_COLLECTOR, MODE_COMMAND, MODES,
                               STEP_FILED, STEP_HOLD, STEP_IDLE,
                               STEP_IDLE_ASK, STEP_OFFER, STEP_SEND,
                               TOPIC_REPLY_ALL, make_query, make_reply,
                               make_request, request_mode, topic_query,
                               topic_reply)

    check("there are exactly two modes and command is the default",
          MODES == (MODE_COMMAND, MODE_COLLECTOR)
          and request_mode(make_request("r", "P2,4R"))[0] == "c")

    # THE MODE IS SIGNED. It changes how the robot behaves, so it is an
    # instruction like the target list and gets the same protection: a mode
    # carried outside the signature could be flipped by anyone on the broker.
    d = Path(tempfile.mkdtemp())
    cloud = Cloud(Store(d / "s"), d / "k")
    cloud.mode = MODE_COLLECTOR
    rid, signed = cloud.build_request("P2,4R")
    body = verify_json(signed, _keys.public_of("cloud", d / "k"))
    check("the mode travels INSIDE the signed request",
          request_mode(body) == MODE_COLLECTOR)
    check("and rewriting it breaks the signature",
          _refuses_mode(signed, d / "k"),
          "a mode outside the signature could be flipped on the wire")
    check("an unknown mode falls back to command, rather than raising",
          request_mode({"mode": "nonsense"}) == MODE_COMMAND,
          "a robot that refused an order naming a mode it had not heard of "
          "would stop working on a Cloud upgrade")

    # --- the Cloud side of the negotiation, with no broker ---------------
    # BOTH DIRECTIONS ARE SIGNED. The handshake was plain at first, on the
    # grounds that a forged `send` only makes the robot offer a reading it
    # was going to offer anyway. That missed `hold`: answered to every
    # offer it fills the outbox, starts dropping the oldest readings, and
    # leaves a robot that reports itself healthy while producing nothing.
    from agri import crypto_ecc as _crypto
    from agri.crypto_ecc import sign_json as _sign
    robot_priv = _keys.ensure("robot", d / "k", generate=False).private_pem

    def r_reply(step, node="n1", **extra):
        """A reply as the real robot sends it: signed with the robot key."""
        return json.dumps(_sign(make_reply(step, node, **extra),
                                robot_priv)).encode()

    def step_of(q):
        """The step inside a SIGNED query."""
        return q["payload"]["step"]

    cloud.on_reply(r_reply(STEP_IDLE, idle=True), "n1")
    check("the Cloud records that the node said it was stopped",
          cloud.node_idle.get("n1") is True)
    cloud.on_reply(r_reply(STEP_IDLE, idle=False), "n1")
    check("and that it said it was moving", cloud.node_idle.get("n1") is False)

    qs = cloud.on_reply(r_reply(STEP_OFFER, label="P2,4R", holding=1), "n1")
    check("an offer is answered with send when the Cloud is receiving",
          [step_of(q) for q in qs] == [STEP_SEND])
    check("and the answer is SIGNED, not a bare dict on the wire",
          all("sig" in q and "payload" in q for q in qs),
          "an unsigned grant is a grant anyone on the broker can write")
    cloud.receiving = False
    qs = cloud.on_reply(r_reply(STEP_OFFER, label="P2,4L", holding=2), "n1")
    check("and with hold when it is not",
          [step_of(q) for q in qs] == [STEP_HOLD],
          "the robot must be told to keep it, not left to time out")
    check("the decision is returned, not published",
          all(isinstance(q, dict) for q in qs),
          "on_reply decides and the caller transmits, so the whole "
          "negotiation is exercisable with no broker in the room")
    check("and the exchange is recorded as dialogue for the dashboard",
          [e["step"] for e in cloud.dialogue][-2:] == [STEP_OFFER, STEP_HOLD])

    # The refusals. An unsigned reply is what an attacker can write; a
    # reply signed by the wrong key is what a decommissioned robot writes.
    before = len(cloud.rejected)
    plain = json.dumps(make_reply(STEP_IDLE, "n1", idle=True)).encode()
    check("an UNSIGNED reply is refused",
          cloud.on_reply(plain, "n1") == []
          and len(cloud.rejected) == before + 1,
          "a forged 'idle' tells the Cloud a moving robot is stopped, and "
          "collector mode then orders it off mid-drive")
    stranger = _crypto.generate_keypair()
    forged = json.dumps(_sign(make_reply(STEP_OFFER, "n1", label="P2,4R"),
                              stranger.private_pem)).encode()
    check("and so is one signed by a key the Cloud does not trust",
          cloud.on_reply(forged, "n1") == [])
    check("and the refusal is COUNTED, not silently dropped",
          any("handshake refused" in r["reason"]
              for r in list(cloud.rejected)[-2:]),
          "a pipeline that hides its rejections runs empty and looks fine")
    stale = _sign(dict(make_reply(STEP_OFFER, "n1", label="P2,4R"),
                       at="2020-01-01T00:00:00Z"), robot_priv)
    check("a correctly signed reply from LAST YEAR is refused too",
          cloud.on_reply(json.dumps(stale).encode(), "n1") == [],
          "a 'hold' captured today and replayed next campaign is the same "
          "denial of service, arriving late")
    cloud.receiving = True

    # --- the robot side --------------------------------------------------
    import agri.robot as R
    saved, R.GRANT_TIMEOUT_S = R.GRANT_TIMEOUT_S, 0.25
    try:
        sent = []

        class _C:
            def publish(self, t, p, **k):
                sent.append((t, json.loads(p) if p else None))

        class _V:
            robot_id = "n1"

            class driver:
                moving = [0.0, 0.0]

                @classmethod
                def velocity(cls):
                    return (cls.moving[0], cls.moving[1], 0.0)

        _V.robot_private_pem = robot_priv
        cloud_pub = _keys.public_of("cloud", d / "k", generate=False)
        link = R.RobotLink(_V(), cloud_pub, _C(), log=lambda m: None)
        check("a stopped robot says so", link.is_still())
        _V.driver.moving = [0.5, 0.0]
        check("and a moving one does not", not link.is_still())
        _V.driver.moving = [0.0, 0.0]

        link.on_query(json.dumps(cloud.query(STEP_IDLE_ASK, "n1")).encode())
        check("the robot answers the idleness question on its reply topic",
              sent and sent[-1][0] == topic_reply("n1")
              and sent[-1][1]["payload"]["step"] == STEP_IDLE)
        check("and its answer is SIGNED with the robot's own key",
              "sig" in sent[-1][1]
              and _crypto.verify_json(sent[-1][1],
                                      _keys.public_of("robot", d / "k",
                                                      generate=False))
              is not None)

        # The forged grant. This is the one that matters: `hold` answered to
        # every offer fills the outbox and the robot never says anything is
        # wrong, because from its side nothing is.
        n0 = len(sent)
        link.on_query(json.dumps(make_query(STEP_HOLD, "n1")).encode())
        check("the robot REFUSES an unsigned query",
              len(sent) == n0 and not link._grant.is_set(),
              "an unsigned 'hold' answered to every offer is a silent "
              "denial of service that costs one MQTT publish")
        link.on_query(json.dumps(
            _sign(make_query(STEP_SEND, "n1"), stranger.private_pem)).encode())
        check("and one signed by a stranger",
              not link._grant.is_set())

        n = len(sent)
        out = link._deliver("P1,1R", {"e": 1}, MODE_COMMAND)
        check("command mode sends straight out, with no negotiation",
              out.endswith("sent")
              and [t for t, _ in sent[n:]] == [f"agri/v1/report/n1"])

        out = link._deliver("P1,2R", {"e": 2}, MODE_COLLECTOR)
        check("collector mode HOLDS the reading when nobody answers",
              out.endswith("held") and len(link.outbox) == 1,
              "a robot that dumped readings at a Cloud that never replied "
              "would make the negotiation decorative")

        def _grant():
            time.sleep(0.05)
            link.on_query(json.dumps(cloud.query(STEP_SEND, "n1")).encode())

        _th.Thread(target=_grant).start()
        n = len(sent)
        out = link._deliver("P1,3R", {"e": 3}, MODE_COLLECTOR)
        got = [p["e"] for t, p in sent[n:] if t.startswith("agri/v1/report/")]
        check("and hands everything over when granted", not link.outbox)
        check("oldest first, so a campaign arrives in the order it happened",
              got == [2, 3], str(got))

        _th.Thread(target=lambda: (
            time.sleep(0.05),
            link.on_query(json.dumps(
                cloud.query(STEP_HOLD, "n1")).encode()))).start()
        out = link._deliver("P1,4R", {"e": 4}, MODE_COLLECTOR)
        check("an explicit hold keeps it too, without waiting for a timeout",
              out.endswith("held") and len(link.outbox) == 1)

        check("the outbox is bounded",
              link.outbox.maxlen == R.OUTBOX_MAX and R.OUTBOX_MAX >= 48,
              "an unbounded queue turns a Cloud outage into a robot that "
              "runs out of memory mid-aisle; 48 is one whole campaign")

        link.on_query(b"{not json")
        check("a malformed query cannot kill the MQTT thread", True)
    finally:
        R.GRANT_TIMEOUT_S = saved

    # --- wiring ----------------------------------------------------------
    srv = (ROOT / "agri" / "cloud" / "server.py").read_text()
    rn = (ROOT / "ros2" / "src" / "agri_robot" / "agri_robot"
          / "robot_node.py").read_text()
    check("the Cloud subscribes to the reply topic",
          "TOPIC_REPLY_ALL" in srv)
    check("the robot subscribes to its OWN query topic, not a wildcard",
          "topic_query(self.robot_id)" in rn,
          "a query names one node; a second robot must not answer this "
          "one's")
    check("the Cloud acknowledges every filed report, in both modes",
          "STEP_FILED" in srv and "query(STEP_FILED" in srv,
          "a node that never hears its reading landed cannot tell a filed "
          "report from a lost one")
    check("collector mode asks whether the robot is stopped BEFORE ordering",
          srv.index("ask_idle") < srv.index("build_request(targets)"),
          "a mission begun mid-move starts its first leg from a stale pose")
    check("a node that does not answer is not sent an order anyway",
          "did not answer; not issuing an order" in srv)
    check("the mode can be switched from the console, mid-session",
          "def set_mode" in srv and 'verb == "mode"' in srv,
          "restarting the Cloud to show the other mode shows a jury two "
          "programs, not one system with two modes")
    check("and from the dashboard, which is what a jury is looking at",
          '"/api/mode"' in srv,
          "a console verb behind the operator's shoulder is not a "
          "demonstration of two modes")
    check("and the Cloud can be told to stop receiving, to show the buffer",
          'verb in ("pause", "resume")' in srv)

    # --- the log reads as a conversation ---------------------------------
    pretty = ROOT / "tools" / "prettylog.py"
    sample = "\n".join(
        f"[robot_node-7] [INFO] [1.0] [agri_robot]: {m}" for m in (
            "robot: Cloud asks if I am stopped -> stopped",
            "robot: accepted 788081147cfd, 48 station(s) in collector mode",
            "robot: parked and still at P1,1R, reading ready -- asking the "
            "Cloud to receive",
            "robot: the Cloud said hold -- keeping P1,2R, 1 reading(s) held",
            "robot: Cloud filed P1,1R",
            "robot: accepted 49b8268b46e8, 1 station(s) in command mode"))
    out = subprocess.run([sys.executable, str(pretty), "--no-colour"],
                         input=sample, capture_output=True, text=True,
                         timeout=60).stdout
    check("the log shows the robot answering the idleness question",
          "I am stopped" in out, out)
    check("and the robot asking permission to send", "may I send?" in out, out)
    check("and the Cloud refusing, with the reading kept on board",
          "KEEPS P1,2R on board" in out, out)
    check("and the mode each mission is running in",
          "collector mode" in out and "command mode" in out, out)
    check("each request opens with a separating rule",
          out.count("── request ") == 2, out)
    check("and the rule names the request, so a line can be traced back",
          "── request 788081147cfd" in out, out)
    src = pretty.read_text()
    check("the two directions are distinguishable without colour",
          '"cloud": "──► "' in src and '"node": "◄── "' in src,
          "a log read over a projector or printed in a report loses the "
          "colour and keeps the arrows")
    check("and they are coloured differently when there is a terminal",
          "cls.cloud, cls.node" in src)


def _refuses_mode(signed: dict, key_dir: Path) -> bool:
    """Rewrite the mode inside a signed request; the signature must fail.

    sign_json puts the payload in as a plain dict, so tampering is one
    assignment -- which is exactly why the mode had to go inside it.
    """
    from agri import keys as _keys
    from agri.crypto_ecc import CryptoError, verify_json

    tampered = json.loads(json.dumps(signed))
    tampered["payload"]["mode"] = "command"
    try:
        verify_json(tampered, _keys.public_of("cloud", key_dir))
    except CryptoError:
        return True
    return False


def check_timing() -> None:
    """When was it asked for, and when was it answered.

    The system used to keep one of the three moments a reading has, so the
    only honest answer to "how long did that sweep take" was to watch a clock
    while it ran. Every check here is about a number that has to survive from
    the request being signed all the way into the CSV, because that file is
    what ends up in the report.
    """
    print("\ntiming: when a request was issued and when it completed")
    from agri import keys                                  # noqa: PLC0415
    from agri.cloud.server import Cloud                    # noqa: PLC0415
    from agri.cloud.store import Store, seconds_between    # noqa: PLC0415
    from agri.robot import RobotLink, SimulatedDriver, Visitor  # noqa: PLC0415
    from agri.sensors import GreenhouseField               # noqa: PLC0415

    check("a duration between two of our own stamps is computed, not guessed",
          seconds_between("2026-08-08T14:00:00Z", "2026-08-08T14:02:30Z") == 150.0,
          str(seconds_between("2026-08-08T14:00:00Z", "2026-08-08T14:02:30Z")))
    check("and a missing or foreign stamp gives None rather than raising",
          seconds_between(None, "2026-08-08T14:00:00Z") is None
          and seconds_between("last tuesday", "2026-08-08T14:00:00Z") is None,
          "visits filed before these fields existed have neither; the "
          "campaign must stay readable across a change to its own format")

    with tempfile.TemporaryDirectory() as d:
        d = Path(d)
        keydir, storedir = d / "keys", d / "store"
        cloud_keys = keys.ensure("cloud", keydir)
        robot_keys = keys.ensure("robot", keydir)
        cloud = Cloud(Store(storedir), keydir)

        class Loopback:
            def publish(self, topic, payload, qos=1, retain=False):
                data = payload.encode() if isinstance(payload, str) else payload
                if topic.startswith("agri/v1/report/"):
                    cloud.on_report(data)
                elif topic.startswith("agri/v1/ack/"):
                    cloud.on_ack(data)

        visitor = Visitor(robot_id="youbot-01",
                          cloud_public_pem=cloud_keys.public_pem,
                          robot_private_pem=robot_keys.private_pem,
                          read_sensors=GreenhouseField().read,
                          driver=SimulatedDriver(seed=3))
        link = RobotLink(visitor, cloud_keys.public_pem, Loopback(),
                         log=lambda s: None)

        targets = ["P1,1R", "P1,1L", "P2,4R"]
        rid, signed = cloud.build_request(targets)
        p = cloud.progress[rid]
        check("a request records when it was issued",
              bool(p["issued_at"]) and p["issued_at"].endswith("Z"),
              str(p.get("issued_at")))
        check("and the stamp it records is the one that was SIGNED",
              p["issued_at"] == signed["payload"]["issued_at"],
              "the Cloud's note of when it asked and the robot's copy of the "
              "order must be the same number, or they are two opinions")
        check("a request in flight has no completion time yet",
              p["completed_at"] is None and p["elapsed_s"] is None)

        link.mission = link.on_request(json.dumps(signed).encode())
        link.run_mission(lambda: 0.0)

        p = cloud.progress[rid]
        check("a finished request records when it completed",
              p["done"] == p["total"] and bool(p["completed_at"]),
              str(p))
        check("and how long it took, in seconds",
              p["elapsed_s"] is not None and p["elapsed_s"] >= 0,
              str(p.get("elapsed_s")))

        # A mission can end WITHOUT filing everything -- an obstacle, a drive
        # error, a robot restarted mid-sweep. The record has to close anyway,
        # or the dashboard shows it running for ever and requests.csv carries
        # a blank cell with nothing to explain it.
        from agri.protocol import make_ack                  # noqa: PLC0415
        rid2, _ = cloud.build_request(["P3,1R", "P3,1L"])
        cloud.on_ack(json.dumps(make_ack(rid2, "youbot-01", "failed",
                                         detail="lidar brake, gave up")).encode())
        q = cloud.progress[rid2]
        check("an ABORTED request is closed too, not left running for ever",
              q["state"] == "failed" and bool(q["completed_at"])
              and q["elapsed_s"] is not None,
              str(q))
        check("and it still says how far it got before it gave up",
              q["done"] == 0 and q["total"] == 2 and "gave up" in q["detail"],
              str(q))

        # MQTT at QoS 1 is allowed to deliver the same message twice, and the
        # store already tolerates that. The completion time must too, or a
        # campaign's end time drifts every time the broker repeats itself.
        was = p["completed_at"]
        envelope, _ = visitor.visit("P1,1R", 0.0)
        cloud.on_report(json.dumps(envelope).encode())
        check("a re-delivered report does not move the completion time",
              cloud.progress[rid]["completed_at"] == was,
              f"{was} became {cloud.progress[rid]['completed_at']}")

        # --- the three clocks, on the visit itself ------------------------
        v = cloud.store.latest_by_label()["P1,1L"]
        check("a stored visit carries the order's time as well as its own",
              v.request_issued_at == cloud.progress[rid]["issued_at"],
              f"visit says {v.request_issued_at!r}")
        check("and when the Cloud filed it",
              bool(v.received_at) and v.received_at.endswith("Z"),
              str(v.received_at))
        check("so end-to-end latency is derivable from the stored line alone",
              v.latency_s is not None and v.latency_s >= 0, str(v.latency_s))
        check("all three survive a round trip through the JSONL",
              type(v).from_json(v.to_json()).latency_s == v.latency_s,
              "the store is append-only text; a field that does not "
              "round-trip is a field that is lost on the next restart")

        # A reading filed with no request behind it (a redelivery after a
        # restart, a standalone run) must still be filed, not refused.
        loose = dict(json.loads(json.dumps(
            {"measurement": {"label": "P3,8L",
                             "timestamp": "2026-08-08T14:00:00Z",
                             "values": {q: 1.0 for q in
                                        ("temperature", "humidity",
                                         "luminosity", "co2", "ph")}},
             "robot": "youbot-01", "request_id": "not-a-request"})))
        filed = cloud.store.add_report(loose)
        check("a report with no request behind it is still filed",
              filed.label == "P3,8L" and filed.request_issued_at is None
              and filed.latency_s is None,
              "the store must not require a request it has never seen")

        # --- the CSVs, which are what actually reaches the report ---------
        rows = cloud.store.export_csv().read_text().strip().splitlines()
        head = rows[0].split(",")
        for col in ("request_issued_at", "measured_at", "received_at",
                    "latency_s"):
            check(f"the measurements CSV names its {col} column", col in head,
                  f"header is {head}")
        check("and 'timestamp' is gone, because there are three of them now",
              "timestamp" not in head,
              "a column called timestamp next to two other times is a column "
              "whose meaning has to be looked up in the source")
        body = dict(zip(head, rows[1].split(",")))
        check("a CSV row really carries the times, not just the headings",
              body["request_issued_at"] and body["received_at"]
              and body["latency_s"], str(body)[:200])

        # Read back with the csv module, not str.split: a station label has a
        # comma in it (P1,1R), so the targets column is legitimately quoted
        # and splitting on commas would tear it apart. A CSV that only its
        # writer can read is not an export.
        import csv as _csv                                  # noqa: PLC0415

        rpath = cloud.export_requests_csv()
        with rpath.open(newline="") as fh:
            rrows = list(_csv.reader(fh))
        check("requests.csv is written beside the measurements",
              rpath.name == "requests.csv" and rpath.parent == storedir)
        check("with one row per request and a header",
              len(rrows) == 1 + len(cloud.progress),
              f"{len(rrows)} rows for {len(cloud.progress)} request(s)")
        for col in ("issued_at", "completed_at", "elapsed_s", "targets"):
            check(f"and it names its {col} column", col in rrows[0], str(rrows[0]))
        rbody = dict(zip(rrows[0], rrows[1]))
        check("the request row says what was asked for and when it was done",
              rbody["issued_at"] and rbody["completed_at"]
              and rbody["targets"] == ";".join(targets),
              str(rbody)[:220])
        check("and a label's own comma survives the export intact",
              "P1,1R" in rbody["targets"].split(";"),
              f"targets came back as {rbody['targets']!r}")

    # The two exports must move together. Exporting one and not the other
    # from the button, the console verb or the shutdown path is how a run
    # ends with half its evidence.
    server = (ROOT / "agri" / "cloud" / "server.py").read_text()
    check("every export path writes BOTH files",
          server.count("export_requests_csv()") >= 3,
          "the dashboard button, the console's csv verb and the shutdown "
          "path each have to write it, or which one you used decides what "
          "you end up with")

    js = (ROOT / "agri" / "cloud" / "dashboard" / "app.js").read_text()
    check("the dashboard shows when a request was issued and when it finished",
          "issued_at" in js and "completed_at" in js and "elapsed_s" in js)
    check("and converts UTC to the reader's own clock",
          "toLocaleTimeString" in js,
          "the stamps travel as UTC because the greenhouse and the Cloud "
          "need not share a time zone; showing that raw makes everyone "
          "reading the page do the arithmetic")
    check("the station detail names all three moments",
          "request_issued_at" in js and "received_at" in js)
    check("and an abandoned request is not reported as a finished one",
          'r.state === "failed" ? "gave up"' in js,
          "a failed request has an end time and a duration like any other; "
          "labelling that 'done' reports a failure as a success in the one "
          "place the operator looks for the answer")


def check_launch_scripts() -> None:
    """Two commands, and everything dies when either one is stopped.

    These are the files the demonstration actually starts, so they are worth
    more checking than the code they start -- a typo here is a demonstration
    that does not begin.
    """
    print("\nthe two launch scripts")
    import subprocess                                      # noqa: PLC0415

    sim = ROOT / "run_sim.sh"
    cloud_sh = ROOT / "run_cloud.sh"
    pretty = ROOT / "tools" / "prettylog.py"

    for p in (sim, cloud_sh, pretty):
        check(f"{p.name} exists", p.exists(), str(p))
        check(f"and {p.name} is executable",
              p.exists() and os.access(p, os.X_OK),
              "chmod +x, or the operator has to know to type bash first")
    for p in (sim, cloud_sh):
        r = subprocess.run(["bash", "-n", str(p)], capture_output=True,
                           text=True)
        check(f"{p.name} is valid bash", r.returncode == 0, r.stderr)
    check("prettylog.py is valid Python", _parses(pretty.read_text()))

    s = sim.read_text()
    c = cloud_sh.read_text()

    # THE mistake, made three times in one afternoon: launching without the
    # virtualenv, watching robot_node die on `No module named 'qrcode'` a
    # second after Gazebo opens, and concluding that requests do not work.
    check("run_sim.sh activates the virtualenv itself",
          "bin/activate" in s and ".venvs/agri" in s,
          "forgetting it kills the robot node one second after Gazebo opens, "
          "and nothing on screen connects the two")
    check("and refuses to start rather than launching without one",
          'die "no virtualenv found' in s,
          "a launch that half-works is worse than one that does not start")
    check("and proves the dependencies are really importable",
          "import qrcode, paho.mqtt.client, cryptography" in s,
          "an activated venv that was never pip-installed fails identically")
    check("run_cloud.sh activates it too",
          "bin/activate" in c and "no virtualenv found" in c)

    # THE BUG THAT STOPPED run_sim.sh DEAD ON ITS FIRST REAL RUN.
    #
    #     install/setup.bash: line 11: COLCON_TRACE: unbound variable
    #
    # ROS's setup.bash and every one colcon generates read variables they do
    # not set, and under `set -u` that kills the shell. The message names a
    # file this project does not own and says nothing about the flag, three
    # lines up in OUR script, that is reading it. Both scripts keep `set -u`
    # -- it is what turns a typo'd variable into an error rather than an
    # empty string -- so the guard is lifted only around other people's files.
    for name, src in (("run_sim.sh", s), ("run_cloud.sh", c)):
        bare = re.findall(r'^\s*(?:\.|source) +"\$(?!1")', src, re.M)
        check(f"{name} sources every setup file through the set -u guard",
              "source_ros()" in src and not bare,
              f"{len(bare)} unguarded source(s): a colcon setup.bash reads "
              "COLCON_TRACE without setting it, and `set -u` makes that fatal")
    check("and the guard puts set -u back afterwards",
          re.search(r"source_ros\(\) \{\s*\n\s*set \+u\b.*?\n\s*set -u\b",
                    s, re.S) is not None,
          "lifting it for the rest of the script would silently undo the "
          "reason for having it")

    # `shift` inside `for arg in "$@"` removes the FIRST argument, not the
    # one being examined, so `./run_sim.sh gui:=false --raw` used to drop
    # gui:=false and hand --raw to ros2 launch, which rejects it. Run the
    # real loop, lifted out of the real file, rather than trusting a reading.
    loop = re.search(r"^PRETTY_ARGS=\(\).*?^done$", s, re.M | re.S)
    check("the argument loop can be lifted out and exercised", bool(loop))
    if loop:
        probe = (loop.group(0)
                 + '\necho "PRETTY:${PRETTY_ARGS[*]-}"'
                   '\necho "LAUNCH:${LAUNCH_ARGS[*]-}"')
        r = subprocess.run(["bash", "-uc", probe, "run_sim.sh",
                            "gui:=false", "--raw", "rviz:=false"],
                           capture_output=True, text=True, timeout=30)
        got = r.stdout
        check("--raw is taken out wherever it appears in the arguments",
              "PRETTY:--raw" in got, f"{got}{r.stderr}")
        check("and every other argument reaches the launch file, in order",
              "LAUNCH:gui:=false rviz:=false" in got,
              f"{got}{r.stderr} -- an argument eaten here is a launch option "
              "that silently does nothing")
        r = subprocess.run(["bash", "-uc", probe, "run_sim.sh"],
                           capture_output=True, text=True, timeout=30)
        check("and no arguments at all is not an error under set -u",
              r.returncode == 0, r.stderr)

    # Anchored on the COMMAND, not on the words: "ros2 launch" also appears
    # in the comment explaining the argument loop, near the top, and matching
    # that made both orderings below look wrong. The same trap is documented
    # over the gz-camera checks; it is worth avoiding twice.
    RUNS_LAUNCH = "stdbuf -oL -eL ros2 launch"
    check("run_sim.sh really does run the launch", RUNS_LAUNCH in s)
    check("run_sim.sh starts the broker BEFORE the launch",
          s.index("mosquitto -p 1883") < s.index(RUNS_LAUNCH),
          "the robot node connects at startup; it retries now, but a broker "
          "that is already there means no error to explain")
    check("and detects one without needing nc, lsof or ss",
          "/dev/tcp/localhost/1883" in s and "/dev/tcp/localhost/1883" in c,
          "a dependency on a networking tool that may not be installed is a "
          "launch script that fails on somebody else's machine")
    check("run_sim.sh only stops a broker it started itself",
          'MOSQ=""' in s and 'kill -TERM "$MOSQ"' in s,
          "killing a mosquitto the operator was already running would take "
          "out whatever else was using it")

    check("run_sim.sh cleans up through Phase B's kill_sim.sh",
          "kill_sim.sh" in s,
          "gz forks the server and the GUI through a ruby wrapper and neither "
          "is in launch's table; that script already names every pattern")
    check("and its cleanup runs on Ctrl-C, on TERM and on any exit",
          "trap cleanup EXIT INT TERM" in s)
    check("and cannot run twice",
          "CLEANED=1" in s,
          "EXIT fires after INT, so an unguarded handler kills everything "
          "twice and prints the report twice")

    # The two scripts stay two scripts. Starting the Cloud from the
    # simulation would quietly undo the one architectural claim this project
    # makes about itself.
    check("run_cloud.sh does not start a simulation",
          "ros2 launch" not in c and "gz sim" not in c,
          "the Cloud is a separate process on a separate machine, or the "
          "broker between them is decoration")
    check("and run_sim.sh does not start a Cloud",
          "agri.cloud.server" not in s and "agri-cloud" not in s)

    # --- the stop-file contract, spelled in two languages ---------------
    from agri import session                                # noqa: PLC0415

    expected = session.stop_file()
    m = re.search(r'^STOP="([^"]+)"', s, re.M)
    check("run_sim.sh declares the stop file the Cloud writes", bool(m), s[:200])
    if m:
        shell = subprocess.run(
            ["bash", "-c", f'echo "{m.group(1)}"'], capture_output=True,
            text=True, env={**os.environ}).stdout.strip()
        check("and the shell and Python spell the same path",
              shell == str(expected),
              f"shell says {shell!r}, agri.session says {str(expected)!r} -- "
              "a contract written down twice is a contract that drifts")
    check("run_sim.sh clears a stale stop file before it starts",
          s.index('rm -f "$STOP"') < s.index(RUNS_LAUNCH),
          "a file left by a SIGKILLed run would stop the next one instantly")
    check("and watches for a new one while it runs",
          'if [ -e "$STOP" ]' in s and 'kill -INT "$$"' in s)

    server = (ROOT / "agri" / "cloud" / "server.py").read_text()
    check("the Cloud writes it when it stops, however it was stopped",
          "ask_simulation_to_stop()" in server
          and server.index("ask_simulation_to_stop()")
          > server.index("def teardown()"),
          "Ctrl-C in the console and QUITTER on the page both go through "
          "teardown; putting it anywhere else makes them differ")
    check("and there is a way to opt out",
          "--keep-sim" in server and "args.keep_sim" in server)
    check("writing it can fail without taking the Cloud down with it",
          "except OSError:" in (ROOT / "agri" / "session.py").read_text(),
          "a Cloud that could not write a hint file has still done its job")

    # --- the log filter, exercised rather than string-matched -----------
    sample = "\n".join([
        "[INFO] [gazebo-1]: process started with pid [1]",
        "[INFO] [robot_node-7]: process started with pid [2]",
        '[gazebo-1] [GUI] [Wrn] [Application.cc:908] [QT] Dialogs.qml:413:17: '
        'QML ToolButton: Binding loop detected for property "implicitHeight"',
        "[gazebo-1] [Msg] Serving entity system service on [/entity/system/add]",
        "[gazebo-1] Warning [Utils.cc:132] XML Element[gz_frame_id], child of "
        "element[sensor], not defined in SDF.",
        "[parameter_bridge-4] [INFO] [1.0] [gz_bridge]: Creating GZ->ROS "
        "Bridge: [/odom (gz.msgs.Odometry) -> /odom (nav_msgs/msg/Odometry)]",
        "[viz_node-5] [WARN] [1.0] [agri_viz]: agri_viz: still no /odom "
        "after 20 s. Usually this means Gazebo is still loading",
        "[viz_node-5] [INFO] [1.0] [agri_viz]: agri_viz: first /odom at "
        "x=-4.58 y=-1.85 after 38 s; map -> base_link is live",
        "[viz_node-5] [INFO] [1.0] [agri_viz]: agri_viz: the warning above "
        "was a slow load, not a fault -- Gazebo took 38 s to come up.",
        "[robot_node-7] [ERROR] [1.0] [agri_robot]: ModuleNotFoundError: "
        "No module named 'qrcode'",
        "[ERROR] [robot_node-7]: process has died [pid 2, exit code 1, "
        "cmd '/x/lib/agri_robot/robot_node --ros-args'].",
        "[robot_node-7] [INFO] [1.0] [agri_robot]: robot: 788081147cfd "
        "reordered to save 272 m (312 -> 40 m, 45 -> 3 gutter crossings)",
        "[robot_node-7] [INFO] [1.0] [agri_robot]: robot: 788081147cfd "
        "finished (48/48), 41.3 m driven",
    ])
    r = subprocess.run([sys.executable, str(pretty), "--no-colour"],
                       input=sample, capture_output=True, text=True, timeout=60)
    out = r.stdout
    check("the log filter runs without dying on real launch output",
          r.returncode == 0, r.stderr[-400:])
    check("it hides Qt binding loops", "Binding loop" not in out, out)
    check("it hides gz service advertisements", "Serving entity" not in out, out)
    check("it hides the gz_frame_id SDF warning", "gz_frame_id" not in out, out)
    check("it keeps the first-odometry line, which decides whether RViz works",
          "first /odom at x=-4.58" in out, out)
    check("and says how long the wait was, not just that it ended",
          "after 38 s" in out, out)
    check("a slow load is a WARNING, not a failure",
          "still loading" in out or "still loading the world" in out, out)
    check("and the retraction survives the filter",
          "slow load, not a fault" in out,
          "the operator has to see the warning taken back, or the log's "
          "last word is a fault that no longer exists")
    check("it keeps a process dying, and names which one",
          "robot_node DIED" in out, out)
    check("it turns a missing virtualenv into the instruction that fixes it",
          "source ~/.venvs/agri/bin/activate" in out,
          "this exact line scrolled past twice and both times the conclusion "
          "drawn was that requests were broken")
    check("it summarises the bridge instead of one line per topic",
          "gz bridge: 1 topics" in out, out)
    check("it keeps the route-optimiser's saving, not just the raw log",
          "reordered: saved 272 m" in out,
          f"{out}\nthis is the number a demonstration of the optimisation "
          "is FOR; found by watching a real run where it finished in a "
          "third of the time and the filtered terminal gave no reason why")
    check("it keeps the distance actually driven",
          "788081147cfd 41.3 m driven" in out, out)
    check("and says how much it hid, so nobody wonders",
          "routine lines hidden" in out, out)
    check("--raw passes everything through unchanged",
          subprocess.run([sys.executable, str(pretty), "--raw"],
                         input=sample, capture_output=True, text=True,
                         timeout=60).stdout.count("Binding loop") == 1,
          "the filter must never be the only copy of its input")
    check("and run_sim.sh keeps the unfiltered log on disk regardless",
          'tee -a "$RAWLOG"' in s and "logs/" in (ROOT / ".gitignore").read_text(),
          "a filtered view with no original behind it is a view you cannot "
          "check")


def check_multinode() -> None:
    """The protocol changes for multi-node support.

    Today there is one mobile robot. The protocol is designed for N fixed
    nodes (ESP) and 1-4 mobile nodes, so the topics, the state and the
    dashboard must already carry those abstractions.
    """
    print("\nmulti-node protocol")
    from agri.protocol import (KIND_FIXED, KIND_MOBILE, TOPIC_ACK_ALL,
                               TOPIC_REPORT_ALL, TOPIC_REQUEST,
                               TOPIC_STATUS_ALL, make_status,
                               node_id_from_topic, topic_ack, topic_report,
                               topic_status)

    check("topics are namespaced by node_id",
          topic_status("bot-1") == "agri/v1/status/bot-1"
          and topic_report("bot-1") == "agri/v1/report/bot-1"
          and topic_ack("bot-1") == "agri/v1/ack/bot-1")
    check("the Cloud subscribes with wildcards",
          TOPIC_STATUS_ALL == "agri/v1/status/+"
          and TOPIC_REPORT_ALL == "agri/v1/report/+"
          and TOPIC_ACK_ALL == "agri/v1/ack/+")
    check("the request topic stays flat (addresses the fleet)",
          TOPIC_REQUEST == "agri/v1/request"
          and "/" not in TOPIC_REQUEST.split("request", 1)[1])
    check("node_id is extractable from a namespaced topic",
          node_id_from_topic("agri/v1/status/bot-1") == "bot-1"
          and node_id_from_topic("agri/v1/report/esp-42") == "esp-42"
          and node_id_from_topic("agri/v1/request") is None)
    check("every status message carries node_kind",
          make_status("bot-1", True)["node_kind"] == KIND_MOBILE
          and make_status("esp-1", True, node_kind=KIND_FIXED)["node_kind"]
              == KIND_FIXED)

    server = (ROOT / "agri" / "cloud" / "server.py").read_text()
    node = (ROOT / "ros2" / "src" / "agri_robot" / "agri_robot"
            / "robot_node.py").read_text()
    robot = (ROOT / "agri" / "robot.py").read_text()
    check("the Cloud subscribes to wildcard topics",
          "TOPIC_STATUS_ALL" in server and "TOPIC_REPORT_ALL" in server
          and "TOPIC_ACK_ALL" in server)
    check("the Cloud stores nodes in a dict, not a single variable",
          "self.nodes:" in server and "dict[str," in server)
    check("the robot publishes on its own subtopic",
          "topic_ack(" in robot and "topic_report(" in robot
          and "topic_status(" in robot)
    check("the robot node's will_set uses a namespaced topic",
          "topic_status(" in node and "node_kind" in node)

    js = (ROOT / "agri" / "cloud" / "dashboard" / "app.js").read_text()
    check("the dashboard reads nodes, not a single robot",
          "S.state.nodes" in js and "S.state.robot" not in js)

    check("the dashboard sends an auth token with API calls",
          "apiUrl(" in js and "token" in js,
          "the dashboard is now accessible from the network; the API "
          "needs a token or anyone can order a measurement")
    check("the dashboard handles a 401 response",
          "401" in js,
          "a missing or wrong token should prompt for the right one, "
          "not show a blank page")
    check("the Cloud accepts a --dashboard-token argument",
          "--dashboard-token" in server,
          "the API endpoint must be protectable")


def _parses(src: str) -> bool:
    import ast                                        # noqa: PLC0415
    try:
        ast.parse(src)
        return True
    except SyntaxError:
        return False


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
        # Every path the summary prints must be a path that exists. This
        # line said visits.jsonl, which the store has never written, so an
        # operator following it finds nothing and concludes the campaign
        # was not filed.
        named = re.findall(r"^  \w+\s+(/\S+)$", out, re.M)
        missing = [n for n in named if not Path(n).exists()]
        check("and every file the summary points at really exists",
              named and not missing, f"named but absent: {missing}")


def check_hygiene() -> None:
    print("\nrepository hygiene")
    gitignore = (ROOT / ".gitignore").read_text()
    check("private keys are gitignored",
          "keys/" in gitignore and "*.pem" in gitignore,
          "a repository that has held a private key has published it")
    # A plain rglob also catches keys/ and demo_run/keys/ once the demo or
    # the robot has actually run -- both are gitignored and are SUPPOSED to
    # exist on disk at that point. What must never happen is one being
    # tracked by git, which is the only thing "published" actually means.
    import subprocess                                  # noqa: PLC0415
    out = subprocess.run(["git", "ls-files", "*.pem"], cwd=ROOT,
                         capture_output=True, text=True, check=False)
    tracked = [line for line in out.stdout.splitlines() if line.strip()]
    check("and none is tracked by git", not tracked,
          f"found {tracked}")
    check("the Phase B project is untouched",
          (ROOT.parent / "src" / "youbot_control").is_dir()
          and (ROOT.parent / "scripts" / "check_regressions.py").exists(),
          "the harvesting robot must keep working")


def main() -> int:
    print("cloud_agri pre-flight checks")
    for fn in (check_labels, check_catalogue, check_crypto, check_replay,
               check_trust, check_broker_auth, check_chain,
               check_ensure, check_plant_camera, check_textures,
               check_on_the_cross,
               check_qr,
               check_sensors, check_telemetry, check_numpy_abi,
               check_mqtt_compat, check_aisles,
               check_vision,
               check_world,
               check_ros_package, check_two_machines, check_dashboard,
               check_dashboard_auth, check_mode_buttons,
               check_session_buttons, check_multinode,
               check_end_to_end, check_timing, check_modes, check_route, check_launch_scripts,
               check_demo, check_hygiene):
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
