"""demo -- the whole chain in one process: no broker, no Gazebo, no network.

    Cloud  ->  signed request  ->  robot  ->  drive, read, photograph,
    QR-encode, seal (ECC)  ->  Cloud  ->  verify, decrypt, check, store,
    dashboard.

WHY THIS EXISTS

Three reasons, and only the first is convenience.

1. A demonstration must not depend on a broker being up, a simulator
   compiling, and a GPU being present, all at the same time, in a room where
   the projector already failed once. This runs anywhere Python runs.

2. It is the same code. The Cloud object, the Visitor, the RobotLink, the
   crypto, the QR codec and the store are the production ones; only two
   things are swapped -- the MQTT client becomes an in-process Bus, and the
   ROS driver becomes SimulatedDriver. So what you watch here is evidence
   about the real system, not a mock-up that resembles it.

3. It makes the invisible parts visible. --explain prints what actually
   travels on the wire for one station: the QR payload as a scanner reads
   it, the sealed envelope with its ciphertext, and the same envelope after
   the Cloud opens it. That is the part a written report cannot show and a
   running simulator hides.

WHAT IT ALSO PROVES

--explain ends by replaying two attacks against the live Cloud: a report
signed by a key that is not the robot's, and a request signed by a key that
is not the Cloud's. Both are refused, out loud. A demo that only shows the
happy path is a demo that has not been tested.
"""

from __future__ import annotations

import argparse
import json
import queue
import threading
import time
from collections import defaultdict
from pathlib import Path
from typing import Callable

from agri import keys
from agri.cloud.server import Cloud, Handler
from agri.cloud.store import Store, summarise
from agri.crypto_ecc import generate_keypair, seal_json, sign_json
from agri.envelope import open_envelope
from agri.labels import all_labels
from agri.measurement import QUANTITIES, Measurement
from agri.protocol import (ALL, QOS, TOPIC_ACK_ALL, TOPIC_REPORT_ALL,
                           TOPIC_REQUEST, TOPIC_STATUS_ALL, expand_targets,
                           make_request, topic_ack)
from agri.robot import RobotLink, SimulatedDriver, Visitor
from agri.sensors import GreenhouseField


class Bus:
    """An MQTT broker's job, minus the network.

    Deliberately the same three-argument publish/subscribe surface paho
    offers, so RobotLink and the Cloud's publish callback cannot tell the
    difference. Delivery is synchronous and in-order, which makes the demo
    reproducible; a real broker is neither, and the system is written not to
    care (every message carries its own request id).

    Supports the MQTT single-level wildcard (+): subscribing to
    agri/v1/report/+ delivers messages published on agri/v1/report/youbot-01.
    """

    def __init__(self) -> None:
        self.subs: dict[str, list[Callable[[bytes], None]]] = defaultdict(list)
        self.counts: dict[str, int] = defaultdict(int)
        self.retained: dict[str, bytes] = {}

    @staticmethod
    def _matches(pattern: str, topic: str) -> bool:
        pp, tp = pattern.split("/"), topic.split("/")
        return len(pp) == len(tp) and all(
            a == "+" or a == b for a, b in zip(pp, tp))

    def subscribe(self, topic: str, fn: Callable[[bytes], None]) -> None:
        self.subs[topic].append(fn)
        for t, data in self.retained.items():
            if self._matches(topic, t):
                fn(data)

    def publish(self, topic: str, payload, qos: int = QOS,
                retain: bool = False) -> None:
        data = payload.encode() if isinstance(payload, str) else payload
        self.counts[topic] += 1
        if retain:
            self.retained[topic] = data
        for pat, fns in self.subs.items():
            if self._matches(pat, topic):
                for fn in fns:
                    fn(data)


def _rule(title: str) -> None:
    print(f"\n\033[36m{title}\033[0m\n" + "─" * 72)


def _wire_sample(envelope: dict, report: dict, qr_text: str) -> None:
    """Show one station's traffic: what a wiretap sees, and what the Cloud
    sees after it holds the private key."""
    sealed = envelope["sealed"]
    _rule("what the plant's numbers look like as a QR payload")
    print(f"  {qr_text}")
    print(f"  {len(qr_text)} characters -- a phone camera reads this off the "
          "screen and gets the readings back")

    _rule("what travels on the wire (this is the whole message)")
    skeleton = {k: v for k, v in envelope.items() if k != "sealed"}
    print("  " + json.dumps(skeleton))
    for k, v in sealed.items():
        text = v if isinstance(v, str) else json.dumps(v)
        one = text.replace("\n", "\\n")
        print(f"  sealed.{k:<20} {one[:58]}{'...' if len(one) > 58 else ''}")
    print("\n  Nothing above says 21.4 degrees, or P2,5R, or shows a leaf. The"
          "\n  readings, the label and the photograph are inside the AES-GCM"
          "\n  ciphertext, whose key was derived by ECDH against the Cloud's"
          "\n  public key and exists nowhere on the wire.")

    _rule("what the Cloud sees after opening it")
    m = report["measurement"]
    print(f"  station   {report['station']['label']}   "
          f"parked {report['parking_error_m']} m from the cross")
    print("  readings  " + "  ".join(
        f"{q.name}={m['values'][q.name]}{q.unit}" for q in QUANTITIES))
    print(f"  photo     {m.get('label', '')} jpeg "
          f"{report['photo']['width']}x{report['photo']['height']}, "
          f"{len(report['photo']['data_b64'])} b64 chars")


def _spread(visits) -> None:
    """Min and max per quantity, with the station that holds each.

    Not an anomaly detector, and labelled so. A station is 'flagged' only
    when a reading is physically impossible (a pH of 14, a sensor that has
    come loose); a dry patch reads a perfectly plausible 52 %RH and is
    invisible to that test. What makes it visible is the CONTRAST with the
    rest of the run, which is what this shows and what the dashboard's
    colour ramp shows. Calling that 'flagged' would blur two different
    claims -- 'this sensor is broken' and 'this plant needs water'.
    """
    visits = list(visits)
    if len(visits) < 2:
        return
    print("  spread   (lowest and highest of this run -- the contrast is what")
    print("            makes a dry patch or a CO2 pocket visible)")
    for q in QUANTITIES:
        vals = [(v.values[q.name], v.label) for v in visits
                if v.values.get(q.name) is not None]
        if not vals:
            continue
        lo, hi = min(vals), max(vals)
        print(f"    {q.name:<11} {lo[0]:>9.{q.decimals}f} {q.unit:<5} at "
              f"{lo[1]:<7}   {hi[0]:>9.{q.decimals}f} {q.unit:<5} at {hi[1]}")


def _attacks(cloud: Cloud, bus: Bus, envelope: dict, report: dict,
             cloud_public_pem) -> None:
    """Replay two attacks against the live Cloud and the live robot.

    Both assume the strongest realistic attacker: someone who has the
    broker's address, has read this source, and knows the Cloud's PUBLIC key
    -- which is public. What they do not have is either private key, and
    that is the whole of the defence.
    """
    _rule("and now the two things the system has to refuse")

    rogue = generate_keypair()
    forged = dict(envelope)
    forged["sealed"] = seal_json(report, cloud_public_pem, rogue.private_pem)
    print("  1. a fabricated report, correctly encrypted to the Cloud,")
    print("     signed by an attacker's key instead of the robot's")
    print("     ->", cloud.on_report(json.dumps(forged).encode()))

    print("  2. a request to go and measure, signed by someone who is")
    print("     not the Cloud")
    order = sign_json(make_request("forged-1", ["P1,1R"]), rogue.private_pem)
    ack_topic = topic_ack("youbot-01")
    before = bus.counts[ack_topic]
    for fn in bus.subs[TOPIC_REQUEST]:
        fn(json.dumps(order).encode())
    moved = bus.counts[ack_topic] > before
    print("     ->", "the robot ACCEPTED IT -- that is a hole" if moved
          else "the robot refused and did not move")


def run(targets: str | list[str], work: Path, explain: bool,
        serve: int | None, seed: int, dwell: float) -> int:
    work.mkdir(parents=True, exist_ok=True)
    key_dir, store_dir = work / "keys", work / "store"

    cloud_keys = keys.ensure("cloud", key_dir)
    robot_keys = keys.ensure("robot", key_dir)
    cloud = Cloud(Store(store_dir), key_dir)
    bus = Bus()
    field = GreenhouseField()

    visitor = Visitor(robot_id="youbot-01",
                      cloud_public_pem=cloud_keys.public_pem,
                      robot_private_pem=robot_keys.private_pem,
                      read_sensors=field.read,
                      driver=SimulatedDriver(seed=seed, dwell=dwell))
    link = RobotLink(visitor, cloud_keys.public_pem, bus,
                     log=lambda s: print(f"  \033[90m{s}\033[0m"))

    # The robot works its queue on its own thread, exactly as it would when
    # driving: the Cloud must stay responsive while a sweep is running, and
    # a demo where the dashboard freezes for forty stations proves the
    # opposite of what it is meant to prove.
    work_q: queue.Queue = queue.Queue()

    def on_request(payload: bytes) -> None:
        if link.on_request(payload) is not None:
            work_q.put(link.mission)

    def worker() -> None:
        while True:
            if work_q.get() is None:
                return
            link.run_mission(lambda: (time.time() % 86400) / 60.0)

    bus.subscribe(TOPIC_REQUEST, on_request)
    bus.subscribe(TOPIC_REPORT_ALL, lambda p: print("  " + cloud.on_report(p)))
    bus.subscribe(TOPIC_ACK_ALL, cloud.on_ack)
    bus.subscribe(TOPIC_STATUS_ALL, cloud.on_status)
    threading.Thread(target=worker, daemon=True).start()

    link.status(True, "ready at the dock")
    labels = expand_targets(targets)
    _rule(f"the Cloud asks for {len(labels)} station(s): "
          + ", ".join(labels[:6]) + (" ..." if len(labels) > 6 else ""))

    rid, signed = cloud.build_request(labels)
    print(f"  request {rid}, signed with the Cloud's private key "
          f"({len(json.dumps(signed))} bytes)")
    t0 = time.time()
    bus.publish(TOPIC_REQUEST, json.dumps(signed))
    # Wait for the worker, with a ceiling: a demo that hangs forever because
    # the robot refused its own request is worse than one that says so.
    deadline = t0 + 60 + len(labels) * max(2.0, dwell * 8)
    while not (link.mission and link.mission.finished):
        if time.time() > deadline:
            print("  demo: the mission did not finish in time -- aborting")
            return 1
        time.sleep(0.02)
    link.status(True, "back at the dock")
    dt = time.time() - t0

    _rule("what the Cloud now holds")
    measured, total = cloud.store.coverage()
    print(f"  {cloud.accepted} report(s) accepted, {len(cloud.rejected)} "
          f"rejected, in {dt:.1f} s")
    print(f"  coverage {measured}/{total} stations")
    stats = summarise(cloud.store.all_visits())
    print(f"  parking  worst {stats['worst_parking_m']} m, "
          f"mean {stats['mean_parking_m']} m "
          f"(tolerance {visitor.park_tolerance:.2f} m)")
    flagged = sorted({v.label for v in cloud.store.all_visits() if v.flags})
    print("  outside physical range: "
          + (", ".join(flagged) or "nothing -- every reading is plausible"))
    _spread(cloud.store.all_visits())
    csv = cloud.store.export_csv()
    print(f"  visits   {store_dir / 'visits.jsonl'}")
    print(f"  export   {csv}")
    print(f"  media    {store_dir / 'photos'} and {store_dir / 'qr'}")

    if explain and cloud.accepted:
        # One extra visit, kept in hand instead of published, purely to show
        # it. Taken from a REAL visit rather than by re-plumbing the
        # internals: it comes out of exactly the code path that produced the
        # ones already filed above.
        env, _note = visitor.visit(labels[0], 12 * 60.0, "sample")
        rep = open_envelope(env, cloud_keys.private_pem, robot_keys.public_pem)
        _wire_sample(env, rep, Measurement.from_dict(rep["measurement"])
                     .to_qr_text())
        _attacks(cloud, bus, env, rep, cloud_keys.public_pem)

    if serve:
        from functools import partial                # noqa: PLC0415
        from http.server import ThreadingHTTPServer  # noqa: PLC0415

        def publish(topic: str, payload: str) -> None:
            bus.publish(topic, payload)

        httpd = ThreadingHTTPServer(("", serve),
                                    partial(Handler, cloud, publish, ""))
        _rule(f"dashboard on http://localhost:{serve}")
        print("  The SURVEY / ROW buttons issue real requests: they are signed,"
              "\n  published, verified by the robot and driven, live."
              "\n  Ctrl-C to stop.")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\ndemo: stopping")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Run the whole Cloud/robot chain offline.")
    ap.add_argument("targets", nargs="?", default="P1,1R,P1,1L,P2,4R,P2,4L,"
                                                  "P3,7R,P3,7L",
                    help="ALL, a plant (P2,5 = both sides), or a comma-separated "
                         "list of plants and stations "
                         "(default: a six-station tour that includes two "
                         "anomalies)")
    ap.add_argument("--work", type=Path, default=Path("demo_run"),
                    help="where keys, store, photos and exports go")
    ap.add_argument("--serve", type=int, nargs="?", const=8088, default=None,
                    metavar="PORT",
                    help="after the tour, serve the dashboard (default 8088)")
    ap.add_argument("--explain", action="store_true",
                    help="show the QR payload, the sealed envelope and the "
                         "two refusals")
    ap.add_argument("--seed", type=int, default=7, help="parking-error seed")
    ap.add_argument("--dwell", type=float, default=0.0,
                    help="seconds of simulated travel per metre-ish, so the "
                         "dashboard can be watched filling up (try 0.4)")
    args = ap.parse_args(argv)

    targets = args.targets
    if targets.strip().upper() != ALL:
        targets = _split_labels(targets)
    return run(targets, args.work, args.explain, args.serve, args.seed,
               args.dwell)


def _split_labels(text: str) -> list[str]:
    """Split "P1,1R,P2,4L" into labels.

    The comma is inside the label AND between labels, which is a genuine
    ambiguity in the professor's notation rather than a design choice we can
    revisit. Resolved by the shape: a label is P<digits>,<digits><R|L>, so a
    comma that follows an R or an L is a separator and any other comma is
    part of a label.

    That heuristic cannot work for a PLANT, which has no R or L to end on:
    "P2,5,P1,1R" is genuinely two readings of the same characters. So a
    semicolon is accepted as an unambiguous separator and wins outright when
    it is present -- which is also what the launch file's targets:= uses.
    """
    if ";" in text:
        return [t.strip() for t in text.split(";") if t.strip()]
    out, buf = [], ""
    for part in text.split(","):
        buf = f"{buf},{part}" if buf else part
        if buf.strip()[-1:].upper() in ("R", "L"):
            out.append(buf.strip())
            buf = ""
    if buf.strip():
        out.append(buf.strip())
    return out or all_labels()


if __name__ == "__main__":
    raise SystemExit(main())
