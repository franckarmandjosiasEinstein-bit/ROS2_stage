"""server -- the Cloud: issues requests, opens reports, serves the dashboard.

It is deliberately one process with two faces:

    an MQTT client   talks to the robot: signs and publishes requests,
                     receives sealed reports, opens them, files them
    an HTTP server   talks to the operator: serves the dashboard and a small
                     JSON API, and turns a button press into a request

WHY NO WEB FRAMEWORK

http.server is in the standard library and this API has five endpoints. A
framework would add a dependency, a version to pin and a deployment story,
and would not make any of those five shorter. The one thing it would buy --
production-grade concurrency -- is not needed by a dashboard with one
operator, and pretending otherwise would be the kind of choice that looks
like engineering and is not.

WHAT THE CLOUD REFUSES

Everything that does not verify, and it says why. A report that fails to
open is not silently dropped: it is counted, the reason is kept, and the
dashboard shows it. A pipeline that hides its rejections is a pipeline that
will one day be receiving nothing and look perfectly healthy.
"""

from __future__ import annotations

import argparse
import json
import mimetypes
import threading
from collections import deque
from datetime import datetime, timezone
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

from agri import keys
from agri.catalogue import all_stations
from agri.cloud.store import Store, summarise
from agri.crypto_ecc import CryptoError, sign_json
from agri.envelope import EnvelopeError, new_request_id, open_envelope
from agri.labels import all_labels, normalise
from agri.measurement import QUANTITIES
from agri.protocol import (ALL, DEFAULT_BROKER, DEFAULT_PORT, QOS,
                           TOPIC_ACK, TOPIC_REPORT, TOPIC_REQUEST,
                           TOPIC_STATUS, ProtocolError, expand_targets,
                           make_request, mqtt_client)
from agri.sensors import ANOMALIES

DASHBOARD = Path(__file__).parent / "dashboard"


class Cloud:
    """State and behaviour, with no transport in it -- so the whole thing can
    be exercised by the test suite without a broker."""

    def __init__(self, store: Store, key_dir: Path) -> None:
        self.store = store
        self.cloud_keys = keys.ensure("cloud", key_dir)
        self.robot_public = keys.public_of("robot", key_dir)
        self.robot_status: dict[str, Any] = {}
        self.progress: dict[str, Any] = {}
        self.rejected: deque[dict[str, Any]] = deque(maxlen=50)
        self.accepted = 0
        self.started = datetime.now(timezone.utc).isoformat(timespec="seconds")

    # ------------------------------------------------------------ outbound
    def build_request(self, targets: str | list[str]) -> tuple[str, dict]:
        """(request_id, signed message). Targets are expanded HERE so the
        robot never has to interpret a wildcard (see protocol.expand_targets)."""
        labels = expand_targets(targets)
        rid = new_request_id()
        msg = make_request(rid, labels)
        self.progress[rid] = {"request_id": rid, "targets": labels,
                              "total": len(labels), "done": 0,
                              "state": "issued", "label": None, "detail": ""}
        return rid, sign_json(msg, self.cloud_keys.private_pem)

    # ------------------------------------------------------------- inbound
    def on_report(self, payload: bytes) -> str:
        """Open, validate and file one report. Returns a one-line verdict."""
        try:
            envelope = json.loads(payload)
        except Exception as exc:                    # noqa: BLE001
            return self._reject("?", f"payload is not JSON ({exc})")
        try:
            report = open_envelope(envelope, self.cloud_keys.private_pem,
                                   self.robot_public)
        except (CryptoError, EnvelopeError) as exc:
            return self._reject(envelope.get("robot", "?"), str(exc))

        visit = self.store.add_report(report)
        self.accepted += 1
        rid = report.get("request_id")
        if rid in self.progress:
            p = self.progress[rid]
            p["done"] = min(p["total"], p["done"] + 1)
            p["label"] = visit.label
            if p["done"] >= p["total"]:
                p["state"] = "complete"
        flag = f"  FLAGGED {','.join(visit.flags)}" if visit.flags else ""
        return (f"{visit.label:7s} filed  "
                + "  ".join(f"{q.name[:4]}={visit.values[q.name]}"
                            for q in QUANTITIES) + flag)

    def _reject(self, robot: str, reason: str) -> str:
        self.rejected.append({"robot": robot, "reason": reason,
                              "at": datetime.now(timezone.utc)
                              .isoformat(timespec="seconds")})
        return f"REJECTED from {robot}: {reason}"

    def on_ack(self, payload: bytes) -> None:
        try:
            d = json.loads(payload)
        except Exception:                            # noqa: BLE001
            return
        rid = d.get("request_id")
        if rid in self.progress:
            self.progress[rid].update(
                state=d.get("state", "?"), label=d.get("label"),
                detail=d.get("detail", ""))
            if d.get("total"):
                self.progress[rid]["total"] = d["total"]

    def on_status(self, payload: bytes) -> None:
        try:
            self.robot_status = json.loads(payload)
        except Exception:                            # noqa: BLE001
            pass

    # ---------------------------------------------------------------- view
    def state(self) -> dict[str, Any]:
        """Everything the dashboard needs, in one call."""
        latest = self.store.latest_by_label()
        stations = []
        for s in all_stations():
            v = latest.get(s.label)
            stations.append({
                "label": s.label, "row": s.row, "plant": s.plant,
                "side": s.side, "x": s.x, "y": s.y,
                "plant_x": s.plant_x, "plant_y": s.plant_y,
                "measured": v is not None,
                "timestamp": v.timestamp if v else None,
                "values": v.values if v else None,
                "flags": v.flags if v else [],
                "velocity": v.velocity if v else None,
                "photo": v.photo_path if v else None,
                "qr": v.qr_path if v else None,
                "parking_error_m": v.parking_error_m if v else None,
            })
        measured, total = self.store.coverage()
        return {
            "stations": stations,
            "quantities": [{"name": q.name, "unit": q.unit, "lo": q.lo,
                            "hi": q.hi, "decimals": q.decimals}
                           for q in QUANTITIES],
            "summary": {**summarise(self.store.all_visits()),
                        "coverage": [measured, total],
                        "accepted": self.accepted,
                        "rejected": len(self.rejected)},
            "robot": self.robot_status,
            "requests": list(self.progress.values())[-8:],
            "rejected": list(self.rejected)[-8:],
            "anomalies": {k: list(v) for k, v in ANOMALIES.items()},
            "since": self.started,
        }


# ------------------------------------------------------------------- HTTP
class Handler(BaseHTTPRequestHandler):
    server_version = "AgriCloud/1.0"

    def __init__(self, cloud: Cloud, publish, *a, **kw) -> None:
        self.cloud, self.publish = cloud, publish
        super().__init__(*a, **kw)

    def log_message(self, fmt, *args) -> None:      # quiet: the MQTT side talks
        pass

    def _send(self, code: int, body: bytes, ctype: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj: Any, code: int = 200) -> None:
        self._send(code, json.dumps(obj).encode(), "application/json")

    def do_GET(self) -> None:                        # noqa: N802
        path = self.path.split("?")[0]
        if path in ("/", "/index.html"):
            return self._file(DASHBOARD / "index.html")
        if path.startswith("/static/"):
            return self._file(DASHBOARD / Path(path[8:]).name)
        if path == "/api/state":
            return self._json(self.cloud.state())
        if path.startswith("/api/history/"):
            label = path[len("/api/history/"):].replace("%2C", ",")
            return self._json([v.to_json()
                               for v in self.cloud.store.history(label)])
        if path.startswith("/media/"):
            return self._file(self.cloud.store.root / path[len("/media/"):])
        self._json({"error": "not found"}, 404)

    def do_POST(self) -> None:                       # noqa: N802
        if self.path != "/api/request":
            return self._json({"error": "not found"}, 404)
        try:
            n = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(n) or b"{}")
            rid, signed = self.cloud.build_request(body.get("targets", ALL))
        except (ProtocolError, ValueError) as exc:
            return self._json({"error": str(exc)}, 400)
        self.publish(TOPIC_REQUEST, json.dumps(signed))
        print(f"cloud: request {rid} -> "
              f"{len(self.cloud.progress[rid]['targets'])} station(s)")
        return self._json({"request_id": rid,
                           "targets": self.cloud.progress[rid]["targets"]})

    def _file(self, path: Path) -> None:
        # Refuse anything that climbs out of the directory it is served from.
        try:
            path = path.resolve()
            roots = (DASHBOARD.resolve(), self.cloud.store.root.resolve())
            if not any(str(path).startswith(str(r)) for r in roots):
                return self._json({"error": "forbidden"}, 403)
            body = path.read_bytes()
        except (OSError, ValueError):
            return self._json({"error": "not found"}, 404)
        ctype = mimetypes.guess_type(str(path))[0] or "application/octet-stream"
        self._send(200, body, ctype)


# ---------------------------------------------------------------- console
CONSOLE_HELP = """  ALL                 measure every station in the greenhouse (48)
  P2,4                measure a PLANT: its right side, then its left
  P2,4R               measure one side of one plant
  P1,3;P3,7           several, separated by semicolons
  status              where the robot is, how fast, and what it is doing
  show P2,4R          the last reading filed for a station
  coverage            which stations have been measured, and which not
  csv                 export everything to store/measurements.csv
  help                this list
  quit                stop the Cloud (the robot keeps running)"""


class Console:
    """The Cloud's own command line: this is where the orders come from.

    The dashboard can issue requests too, and a browser is the nicer way to
    watch one arrive. But the operator's seat is here, in the process that
    holds the private key, and being able to type a plant's name and watch
    the robot leave is the whole demonstration. Before this existed, the only
    way to order a measurement from a terminal was to start a SECOND Cloud
    with --request: a confusing thing to explain and a worse thing to do.

    Deliberately not a REPL with a framework behind it. input(), a handful of
    verbs, and every unrecognised line treated as a set of targets -- because
    a station or a plant is what the operator types nine times out of ten,
    and making them prefix it with a verb would be ceremony.
    """

    def __init__(self, cloud: Cloud, publish) -> None:
        self.cloud, self.publish = cloud, publish

    def run(self) -> None:
        print(CONSOLE_HELP)
        while True:
            try:
                line = input("\ncloud> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                return
            if not line:
                continue
            verb, _, rest = line.partition(" ")
            verb = verb.lower()
            if verb in ("quit", "exit", "q"):
                return
            try:
                if verb in ("help", "?"):
                    print(CONSOLE_HELP)
                elif verb == "status":
                    self.status()
                elif verb == "coverage":
                    self.coverage()
                elif verb == "show":
                    self.show(rest.strip())
                elif verb == "csv":
                    print(f"  exported {self.cloud.store.export_csv()}")
                else:
                    self.request(line)
            except Exception as exc:                 # noqa: BLE001
                # An operator's typo must not take the Cloud down mid-run.
                print(f"  {type(exc).__name__}: {exc}")

    # -------------------------------------------------------------- verbs
    def request(self, text: str) -> None:
        targets = ([t.strip() for t in text.split(";") if t.strip()]
                   if ";" in text else text)
        rid, signed = self.cloud.build_request(targets)
        labels = self.cloud.progress[rid]["targets"]
        self.publish(TOPIC_REQUEST, json.dumps(signed))
        print(f"  request {rid} signed and published -> {len(labels)} "
              f"station(s): {', '.join(labels[:8])}"
              + (" ..." if len(labels) > 8 else ""))

    def status(self) -> None:
        st = self.cloud.robot_status
        if not st:
            print("  no robot has reported yet -- is the robot node running?")
        else:
            p, v = st.get("pose", {}), st.get("velocity", {})
            print(f"  robot   {st.get('robot')}  "
                  f"{'online' if st.get('online') else 'OFFLINE'}"
                  f"  ({st.get('note', '')})")
            print(f"  at      x={p.get('x')}  y={p.get('y')}  "
                  f"yaw={p.get('yaw_deg')} deg")
            print(f"  moving  {v.get('speed', '?')} m/s"
                  f"   (vx={v.get('vx')} vy={v.get('vy')} wz={v.get('wz')})")
            print(f"  said    {st.get('at')}")
        for p in list(self.cloud.progress.values())[-3:]:
            print(f"  request {p['request_id']}  {p['state']:<9} "
                  f"{p['done']}/{p['total']}  {p.get('label') or ''}")
        if self.cloud.rejected:
            print(f"  REJECTED {len(self.cloud.rejected)} report(s); last: "
                  f"{self.cloud.rejected[-1]['reason']}")

    def coverage(self) -> None:
        measured, total = self.cloud.store.coverage()
        seen = set(self.cloud.store.latest_by_label())
        missing = [lab for lab in all_labels() if lab not in seen]
        print(f"  {measured}/{total} stations measured")
        if missing:
            print(f"  not yet: {', '.join(missing[:12])}"
                  + (f" ... and {len(missing) - 12} more"
                     if len(missing) > 12 else ""))

    def show(self, label: str) -> None:
        if not label:
            print("  show what? for example:  show P2,4R")
            return
        v = self.cloud.store.latest_by_label().get(normalise(label))
        if v is None:
            print(f"  nothing filed for {label} yet")
            return
        print(f"  {v.label}   {v.timestamp}")
        for q in QUANTITIES:
            print(f"    {q.name:<12} {v.values[q.name]:>9} {q.unit}")
        print(f"    parked       {v.parking_error_m} m from the cross")
        if v.pose:
            print(f"    position     x={v.pose.get('x')} y={v.pose.get('y')} "
                  f"yaw={v.pose.get('yaw_deg')} deg")
        if v.velocity:
            print(f"    speed        {v.velocity.get('speed')} m/s   "
                  f"(vx={v.velocity.get('vx')} vy={v.velocity.get('vy')} "
                  f"wz={v.velocity.get('wz')})")
        if v.flags:
            print(f"    FLAGGED      {', '.join(v.flags)}")
        print(f"    photo        {v.photo_path}")
        print(f"    qr           {v.qr_path}")


# ------------------------------------------------------------------- main
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Smart-agriculture Cloud")
    ap.add_argument("--broker", default=DEFAULT_BROKER)
    ap.add_argument("--broker-port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--http-port", type=int, default=8088)
    ap.add_argument("--store", type=Path, default=Path("store"))
    ap.add_argument("--keys", type=Path, default=Path("keys"))
    ap.add_argument("--request", metavar="TARGETS",
                    help="issue one request at startup: ALL, a plant (P2,5, both "
                         "sides), a single station (P2,5R), or a "
                         "comma-separated list. Then keep running.")
    args = ap.parse_args(argv)

    cloud = Cloud(Store(args.store), args.keys)
    client = mqtt_client("agri-cloud")

    def publish(topic: str, payload: str) -> None:
        client.publish(topic, payload, qos=QOS)

    def on_connect(cl, _u, _f, rc, *_):
        if rc != 0:
            print(f"cloud: broker refused the connection (rc={rc})")
            return
        for topic in (TOPIC_REPORT, TOPIC_ACK, TOPIC_STATUS):
            cl.subscribe(topic, qos=QOS)
        print(f"cloud: connected to {args.broker}:{args.broker_port}, "
              f"listening on {TOPIC_REPORT}")
        if args.request:
            rid, signed = cloud.build_request(
                args.request if args.request.upper() == ALL
                else [t for t in args.request.split(",") if t.strip()])
            publish(TOPIC_REQUEST, json.dumps(signed))
            print(f"cloud: request {rid} -> {len(cloud.progress[rid]['targets'])}"
                  " station(s)")

    def on_message(_cl, _u, msg):
        if msg.topic == TOPIC_REPORT:
            print("cloud:", cloud.on_report(msg.payload))
        elif msg.topic == TOPIC_ACK:
            cloud.on_ack(msg.payload)
        elif msg.topic == TOPIC_STATUS:
            cloud.on_status(msg.payload)

    client.on_connect, client.on_message = on_connect, on_message
    try:
        client.connect(args.broker, args.broker_port, keepalive=30)
    except OSError as exc:
        print(f"cloud: cannot reach the broker at {args.broker}:"
              f"{args.broker_port} ({exc}).\n"
              "       Start one:  mosquitto -p 1883 -v")
        return 1
    client.loop_start()

    httpd = ThreadingHTTPServer(("", args.http_port),
                                partial(Handler, cloud, publish))
    print(f"cloud: dashboard on http://localhost:{args.http_port}")
    print(f"cloud: {len(all_labels())} stations known, "
          f"{cloud.store.coverage()[0]} already measured")
    threading.Thread(target=httpd.serve_forever, daemon=True).start()

    # The console owns the main thread. The HTTP server and the MQTT client
    # both run behind it, so orders can be typed here while the dashboard is
    # open and the robot is driving -- which is the arrangement the whole
    # thing is meant to demonstrate.
    try:
        Console(cloud, publish).run()
    finally:
        print("cloud: stopping")
        try:
            print(f"cloud: exported {cloud.store.export_csv()}")
        except Exception as exc:                     # noqa: BLE001
            print(f"cloud: could not export ({exc})")
        client.loop_stop()
        client.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
