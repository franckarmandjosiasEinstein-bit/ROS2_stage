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
import os
import threading
import time
from collections import deque
from datetime import datetime, timezone
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

from agri import keys, session
from agri.catalogue import all_stations
from agri.cloud.store import Store, seconds_between, summarise
from agri.crypto_ecc import CryptoError, sign_json
from agri.envelope import EnvelopeError, new_request_id, open_envelope
from agri.labels import all_labels, normalise
from agri.measurement import QUANTITIES, utc_now
from agri.protocol import (ALL, DEFAULT_BROKER, DEFAULT_PORT, MODE_COLLECTOR,
                           MODE_COMMAND, MODES, QOS, STEP_FILED, STEP_HOLD,
                           STEP_IDLE, STEP_IDLE_ASK, STEP_OFFER, STEP_SEND,
                           TOPIC_ACK_ALL, TOPIC_REPLY_ALL, TOPIC_REPORT_ALL,
                           TOPIC_REQUEST, TOPIC_STATUS_ALL, ProtocolError,
                           expand_targets, make_query, make_request,
                           mqtt_client, node_id_from_topic, topic_query)
from agri.sensors import ANOMALIES
from agri.survey import energy_wh

DASHBOARD = Path(__file__).parent / "dashboard"


class Cloud:
    """State and behaviour, with no transport in it -- so the whole thing can
    be exercised by the test suite without a broker."""

    def __init__(self, store: Store, key_dir: Path) -> None:
        self.store = store
        # Both pairs together into an empty directory, or nothing at all --
        # see keys.bootstrap. generate=False on the ROBOT's public key is
        # what stops a Cloud running on its own machine from inventing a
        # robot identity and then rejecting the real robot's every report
        # without either side saying why.
        keys.bootstrap(key_dir)
        self.cloud_keys = keys.ensure("cloud", key_dir, generate=False)
        self.robot_public = keys.public_of("robot", key_dir, generate=False)
        self.nodes: dict[str, dict[str, Any]] = {}
        self.progress: dict[str, Any] = {}
        self.rejected: deque[dict[str, Any]] = deque(maxlen=50)
        self.accepted = 0
        self.started = datetime.now(timezone.utc).isoformat(timespec="seconds")
        #: command or collector. Held here rather than on the command line
        #: so it can be switched mid-session, which is the only way to show
        #: both in one demonstration.
        self.mode = MODE_COMMAND
        #: Set false to make the Cloud answer 'hold' to an offer. Exists so
        #: the buffering can be DEMONSTRATED rather than described: the
        #: robot keeps measuring, the readings pile up on board, and they
        #: all arrive the moment this goes true again.
        self.receiving = True
        #: What the last handshake said, newest last, for the dashboard.
        self.dialogue: deque[dict[str, Any]] = deque(maxlen=40)
        #: Last answer to "are you stopped?", per node.
        self.node_idle: dict[str, bool] = {}
        self.node_busy: dict[str, bool] = {}
        self._idle_seen = threading.Event()

    def ask_idle(self, node: str, publish, timeout: float = 3.0) -> bool | None:
        """Ask a node whether it is stopped, and wait for the answer.

        True / False as answered, None if it did not answer at all -- three
        outcomes, because "the robot says it is moving" and "the robot is
        not there" call for different things to be printed and only one of
        them is worth retrying.
        """
        self._idle_seen.clear()
        self.node_idle.pop(node, None)
        self.say("cloud", f"asking {node}: are you stopped?", STEP_IDLE_ASK)
        publish(topic_query(node), json.dumps(make_query(STEP_IDLE_ASK, node)))
        if not self._idle_seen.wait(timeout):
            self.say("cloud", f"{node} did not answer", "timeout")
            return None
        return self.node_idle.get(node)

    def say(self, who: str, text: str, step: str = "") -> None:
        self.dialogue.append({"who": who, "text": text, "step": step,
                              "at": utc_now()})

    # ------------------------------------------------------------ outbound
    def build_request(self, targets: str | list[str]) -> tuple[str, dict]:
        """(request_id, signed message). Targets are expanded HERE so the
        robot never has to interpret a wildcard (see protocol.expand_targets).

        The order's own timestamp comes out of make_request rather than being
        taken again here: it is the moment that gets SIGNED and travels to the
        robot, so the Cloud's record of when it asked and the robot's record
        of what it was asked are the same number by construction.
        """
        labels = expand_targets(targets)
        rid = new_request_id()
        msg = make_request(rid, labels, mode=self.mode)
        self.progress[rid] = {"request_id": rid, "targets": labels,
                              "total": len(labels), "done": 0,
                              "state": "issued", "label": None, "detail": "",
                              "mode": self.mode,
                              "issued_at": msg["issued_at"],
                              "updated_at": msg["issued_at"],
                              "completed_at": None, "elapsed_s": None,
                              # None, not 0: until the robot says, the
                              # distance is unknown rather than nothing.
                              "metres": None, "planned_m": None}
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

        rid = report.get("request_id")
        p = self.progress.get(rid)
        visit = self.store.add_report(
            report, request_issued_at=p["issued_at"] if p else None)
        self.accepted += 1
        if p is not None:
            p["done"] = min(p["total"], p["done"] + 1)
            p["label"] = visit.label
            p["updated_at"] = utc_now()
            if p["done"] >= p["total"]:
                p["state"] = "complete"
                # Set once. A re-delivered report for a finished request must
                # not move the completion time forward -- MQTT at QoS 1 makes
                # that a normal event, not a fault, and a campaign whose end
                # time drifts every time the broker repeats itself is worse
                # than one with no end time at all.
                if p["completed_at"] is None:
                    p["completed_at"] = p["updated_at"]
                    p["elapsed_s"] = seconds_between(p["issued_at"],
                                                      p["completed_at"])
        self.say("cloud", f"received {visit.label}, opened, verified, "
                 "filed", STEP_FILED)
        flag = f"  FLAGGED {','.join(visit.flags)}" if visit.flags else ""
        took = f"  [+{visit.latency_s:.0f}s]" if visit.latency_s is not None else ""
        return (f"{visit.label:7s} filed  "
                + "  ".join(f"{q.name[:4]}={visit.values[q.name]}"
                            for q in QUANTITIES) + flag + took)

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
        p = self.progress.get(rid)
        if p is None:
            return
        p.update(state=d.get("state", "?"), label=d.get("label"),
                 detail=d.get("detail", ""),
                 updated_at=d.get("at") or utc_now())
        if d.get("total"):
            p["total"] = d["total"]
        # Distance arrives on the closing ack, and only from a robot that
        # counts it. `is not None` rather than a truth test: a mission that
        # genuinely drove 0.0 m -- one station the robot was already parked
        # on -- is a real answer and must not be dropped for looking falsy.
        for f in ("metres", "planned_m"):
            if d.get(f) is not None:
                p[f] = d[f]
        # A mission that ENDS without filing every report still ends, and the
        # record has to close or the dashboard shows it running for ever and
        # requests.csv carries a blank completion with nothing to explain it.
        # An aborted sweep took however long it took before it gave up, and
        # that is a more useful number than an empty cell.
        if d.get("state") in ("finished", "failed") and p["completed_at"] is None:
            p["completed_at"] = p["updated_at"]
            p["elapsed_s"] = seconds_between(p["issued_at"], p["completed_at"])

    # ------------------------------------------------------- handshake
    def on_reply(self, payload: bytes, node_id: str = "") -> list[dict]:
        """A node answering, or asking to send. Returns queries to publish.

        Returns them rather than publishing them, so the whole negotiation
        can be exercised by the test suite with no broker: the transport is
        the caller's business and the decision is this method's.
        """
        try:
            d = json.loads(payload)
        except Exception:                            # noqa: BLE001
            return []
        nid = node_id or d.get("node", "?")
        step = d.get("step")

        if step == STEP_IDLE:
            self.node_idle[nid] = bool(d.get("idle"))
            self.node_busy[nid] = bool(d.get("busy"))
            self._idle_seen.set()
            self.say(nid, "stopped, ready for an order" if d.get("idle")
                     else "still moving", STEP_IDLE)
            return []

        if step == STEP_OFFER:
            label = d.get("label", "?")
            held = d.get("holding", 1)
            self.say(nid, f"parked at {label}, reading ready"
                     + (f" ({held} held)" if held > 1 else ""), STEP_OFFER)
            if self.receiving:
                self.say("cloud", f"ready -- send {label}", STEP_SEND)
                return [make_query(STEP_SEND, nid, label=label)]
            # Not a failure. The robot keeps the reading and carries on.
            self.say("cloud", f"not ready -- hold {label}", STEP_HOLD)
            return [make_query(STEP_HOLD, nid, label=label)]
        return []

    def on_status(self, payload: bytes, node_id: str = "") -> None:
        try:
            d = json.loads(payload)
        except Exception:                            # noqa: BLE001
            return
        nid = node_id or d.get("robot", "?")
        self.nodes[nid] = d

    # -------------------------------------------------------------- export
    #: What was asked for and when it was answered. The measurements CSV is
    #: about readings; this one is about ORDERS, and the two questions have
    #: different rows -- one request covers 48 stations, and "when did the
    #: sweep finish" is not answerable by looking at 48 lines and taking a
    #: maximum, because a sweep that never finished has no line to look at.
    #: driven_m is MEASURED (the robot's odometry, integrated); planned_m is
    #: what the order it chose was expected to cost; the three energy columns
    #: are DERIVED from driven_m and elapsed_s by agri.survey's stated model,
    #: whose constants are assumptions and are named as such where they live.
    #: Kept apart in the header so a reader can tell which is which without
    #: opening the source.
    REQUEST_COLUMNS = ["request_id", "issued_at", "completed_at", "elapsed_s",
                       "state", "done", "total",
                       "driven_m", "planned_m",
                       "energy_wh_est", "idle_wh_est", "drive_wh_est",
                       "targets"]

    def export_requests_csv(self, path: Path | None = None) -> Path:
        import csv                                          # noqa: PLC0415

        path = Path(path or self.store.root / "requests.csv")
        with path.open("w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(self.REQUEST_COLUMNS)
            for p in self.progress.values():
                metres, elapsed = p.get("metres"), p.get("elapsed_s")
                # Energy needs BOTH a distance and a duration. With either
                # missing the cells stay empty rather than being filled with
                # a number computed from a zero that was never measured.
                e = (energy_wh(metres, elapsed)
                     if metres is not None and elapsed is not None else {})
                w.writerow([p["request_id"], p.get("issued_at", ""),
                            p.get("completed_at") or "",
                            "" if elapsed is None else elapsed,
                            p.get("state", ""), p.get("done", 0),
                            p.get("total", 0),
                            "" if metres is None else metres,
                            "" if p.get("planned_m") is None else p["planned_m"],
                            e.get("total_wh", ""), e.get("idle_wh", ""),
                            e.get("drive_wh", ""),
                            ";".join(p.get("targets", []))])
        return path

    # ---------------------------------------------------------------- view
    @staticmethod
    def _with_energy(p: dict[str, Any]) -> dict[str, Any]:
        """A progress record plus its energy estimate, if one can be made.

        Derived on the way out rather than stored, so the model can change
        without rewriting anything already recorded -- and so the stored
        record keeps only what was measured. The key is absent, not zero,
        when either input is missing; the dashboard tests for null.
        """
        metres, elapsed = p.get("metres"), p.get("elapsed_s")
        if metres is None or elapsed is None:
            return p
        e = energy_wh(metres, elapsed)
        return {**p, "energy_wh": e["total_wh"],
                "energy_idle_wh": e["idle_wh"], "energy_drive_wh": e["drive_wh"]}

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
                "request_issued_at": v.request_issued_at if v else None,
                "received_at": v.received_at if v else None,
                "latency_s": v.latency_s if v else None,
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
            "nodes": dict(self.nodes),
            "requests": [self._with_energy(p)
                         for p in list(self.progress.values())[-8:]],
            "rejected": list(self.rejected)[-8:],
            "anomalies": {k: list(v) for k, v in ANOMALIES.items()},
            "since": self.started,
        }


# ------------------------------------------------------------------- HTTP
class Handler(BaseHTTPRequestHandler):
    server_version = "AgriCloud/1.0"

    def __init__(self, cloud: Cloud, publish, auth_token: str,
                 shutdown=None, *a, **kw) -> None:
        self.cloud, self.publish = cloud, publish
        self.auth_token = auth_token
        # None when nothing owns the process's lifetime -- the offline demo
        # and the tests both serve the dashboard without being allowed to
        # kill the interpreter out from under their caller.
        self.shutdown_cloud = shutdown
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

    def _check_auth(self) -> bool:
        """Return True if the request is authorised, or send 401."""
        if not self.auth_token:
            return True
        from urllib.parse import parse_qs, urlparse  # noqa: PLC0415
        qs = parse_qs(urlparse(self.path).query)
        token = qs.get("token", [""])[0]
        if not token:
            auth = self.headers.get("Authorization", "")
            if auth.startswith("Bearer "):
                token = auth[7:]
        if token == self.auth_token:
            return True
        self._json({"error": "unauthorized"}, 401)
        return False

    def do_GET(self) -> None:                        # noqa: N802
        path = self.path.split("?")[0]
        if path in ("/", "/index.html"):
            return self._file(DASHBOARD / "index.html")
        if path in ("/table", "/table.html"):
            return self._file(DASHBOARD / "table.html")
        if path.startswith("/static/"):
            return self._file(DASHBOARD / Path(path[8:]).name)
        if path == "/api/state":
            if not self._check_auth():
                return
            return self._json(self.cloud.state())
        if path.startswith("/api/history/"):
            if not self._check_auth():
                return
            label = path[len("/api/history/"):].split("?")[0].replace("%2C", ",")
            return self._json([v.to_json()
                               for v in self.cloud.store.history(label)])
        if path.startswith("/media/"):
            # Photographs, QR images and the CSV export are measurements, so
            # they are behind the same token as the API that describes them.
            if not self._check_auth():
                return
            name = path[len("/media/"):].split("?")[0]
            return self._file(self.cloud.store.root / name)
        self._json({"error": "not found"}, 404)

    def do_POST(self) -> None:                       # noqa: N802
        path = self.path.split("?")[0]
        if path not in ("/api/request", "/api/export", "/api/quit"):
            return self._json({"error": "not found"}, 404)
        if not self._check_auth():
            return
        if path == "/api/export":
            return self._export()
        if path == "/api/quit":
            return self._quit()
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

    def _export(self) -> None:
        """Write both CSVs and say where they landed, on disk and over HTTP.

        The same export the console's `csv` verb does and the same one the
        shutdown path does, so a run stopped from the browser and a run
        stopped with Ctrl-C leave identical files behind.
        """
        try:
            path = self.cloud.store.export_csv()
            plants = self.cloud.store.export_plants_csv()
            requests = self.cloud.export_requests_csv()
        except OSError as exc:
            return self._json({"error": str(exc)}, 500)
        measured, total = self.cloud.store.coverage()
        print(f"cloud: exported {path}, {plants} and {requests}")
        return self._json({"path": str(path), "url": "/media/" + path.name,
                           "plants_path": str(plants),
                           "plants_url": "/media/" + plants.name,
                           "requests_path": str(requests),
                           "requests_url": "/media/" + requests.name,
                           "requests": len(self.cloud.progress),
                           "visits": len(self.cloud.store.all_visits()),
                           "measured": measured, "total": total})

    def _quit(self) -> None:
        """Stop the Cloud from the dashboard.

        The reply goes out BEFORE the process starts dying, or the browser
        gets a connection reset and reports a failure for a shutdown that in
        fact worked. The robot is deliberately left running: it holds its own
        keys and its own mission, and killing it from a web page is not
        something the Cloud is entitled to do.
        """
        if self.shutdown_cloud is None:
            return self._json({"error": "this Cloud cannot be stopped "
                                        "from the dashboard"}, 403)
        self._json({"stopping": True})
        try:
            self.wfile.flush()
        except OSError:
            pass
        self.shutdown_cloud()

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
  mode collector      the Cloud runs the campaign: it polls the robot, waits
                      until it is stopped, then negotiates every handover
  mode command        the operator asks, the Cloud relays (the default)
  pause / resume      refuse or accept handovers, to show the robot holding
                      readings on board while the Cloud is busy
  csv                 export store/measurements.csv and store/requests.csv
  help                this list
  quit                stop the Cloud AND the simulation with it"""


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
                elif verb == "mode":
                    self.set_mode(rest.strip())
                elif verb in ("pause", "resume"):
                    self.cloud.receiving = verb == "resume"
                    print(f"  the Cloud is {'' if self.cloud.receiving else 'NOT '}"
                          "receiving; the robot "
                          + ("will hand over anything it held"
                             if self.cloud.receiving
                             else "will hold its readings on board"))
                elif verb == "csv":
                    print(f"  exported {self.cloud.store.export_csv()}")
                    print(f"  exported {self.cloud.store.export_plants_csv()}")
                    print(f"  exported {self.cloud.export_requests_csv()}")
                else:
                    self.request(line)
            except Exception as exc:                 # noqa: BLE001
                # An operator's typo must not take the Cloud down mid-run.
                print(f"  {type(exc).__name__}: {exc}")

    # -------------------------------------------------------------- verbs
    def request(self, text: str) -> None:
        targets = ([t.strip() for t in text.split(";") if t.strip()]
                   if ";" in text else text)

        # COLLECTOR MODE ASKS FIRST. The order is identical either way; the
        # difference is that here the Cloud will not issue one to a robot
        # that is still rolling. A mission begun mid-move starts its first
        # leg from a pose that is already stale, and the whole point of the
        # mode is that the Cloud runs the campaign rather than the operator.
        if self.cloud.mode == MODE_COLLECTOR:
            node = next(iter(self.cloud.nodes), None)
            if node is None:
                print("  no node has reported yet -- nothing to ask")
                return
            answer = self.cloud.ask_idle(node, self.publish)
            if answer is None:
                print(f"  {node} did not answer; not issuing an order")
                return
            if not answer:
                print(f"  {node} says it is still moving; try again when "
                      "it has stopped")
                return
            print(f"  {node} confirms it is stopped")

        rid, signed = self.cloud.build_request(targets)
        labels = self.cloud.progress[rid]["targets"]
        self.publish(TOPIC_REQUEST, json.dumps(signed))
        print(f"  request {rid} signed and published -> {len(labels)} "
              f"station(s): {', '.join(labels[:8])}"
              + (" ..." if len(labels) > 8 else ""))

    def set_mode(self, name: str) -> None:
        """Switch between command and collector, mid-session.

        Mid-session rather than at startup, because showing both in one
        demonstration is the point: a jury that has to watch the Cloud be
        restarted to see the second mode has been shown two programs, not
        one system with two modes.
        """
        want = {"commande": MODE_COMMAND, "command": MODE_COMMAND,
                "collecteur": MODE_COLLECTOR,
                "collector": MODE_COLLECTOR}.get(name.lower())
        if want is None:
            print(f"  mode is {self.cloud.mode}. Say: mode command | "
                  "mode collector")
            return
        self.cloud.mode = want
        print(f"  mode -> {want}")
        if want == MODE_COLLECTOR:
            print("    the Cloud asks the robot if it is stopped before "
                  "ordering,")
            print("    and the robot asks permission before sending each "
                  "reading.")
            print("    'pause' makes the Cloud refuse; the robot holds them "
                  "on board.")

    def status(self) -> None:
        if not self.cloud.nodes:
            print("  no node has reported yet -- is the robot running?")
        for nid, st in self.cloud.nodes.items():
            kind = st.get("node_kind", "?")
            p, v = st.get("pose", {}), st.get("velocity", {})
            print(f"  node    {nid} ({kind})  "
                  f"{'online' if st.get('online') else 'OFFLINE'}"
                  f"  ({st.get('note', '')})")
            if p:
                print(f"  at      x={p.get('x')}  y={p.get('y')}  "
                      f"yaw={p.get('yaw_deg')} deg")
            if v:
                print(f"  moving  {v.get('speed', '?')} m/s"
                      f"   (vx={v.get('vx')} vy={v.get('vy')} "
                      f"wz={v.get('wz')})")
            print(f"  said    {st.get('at')}")
        for p in list(self.cloud.progress.values())[-3:]:
            print(f"  request {p['request_id']}  {p['state']:<9} "
                  f"{p['done']}/{p['total']}  {p.get('label') or ''}")
            when = f"          issued  {p.get('issued_at', '?')}"
            if p.get("completed_at"):
                when += (f"   completed {p['completed_at']}"
                         f"   ({p['elapsed_s']:.0f} s)")
            else:
                waiting = seconds_between(p.get("issued_at"), utc_now())
                when += ("   still running"
                         + (f" ({waiting:.0f} s so far)"
                            if waiting is not None else ""))
            print(when)
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
def _my_address() -> str:
    """This machine's address on the network the broker is reached over.

    Not the hostname, which resolves to 127.0.1.1 on a stock Debian and
    would send anyone who reads it back to their own machine. A UDP socket
    "connected" to an outside address sends nothing, but does make the
    kernel pick the interface it WOULD use, and its local address is the one
    the robot and the browser can actually reach.
    """
    import socket                                        # noqa: PLC0415
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.0.2.1", 9))       # TEST-NET-1: routed nowhere
        return s.getsockname()[0]
    except OSError:
        return "localhost"                # no network at all; say so honestly
    finally:
        s.close()


def main(argv: list[str] | None = None) -> int:
    import secrets                                       # noqa: PLC0415

    ap = argparse.ArgumentParser(description="Smart-agriculture Cloud")
    ap.add_argument("--broker", default=DEFAULT_BROKER)
    ap.add_argument("--broker-port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--http-port", type=int, default=8088)
    ap.add_argument("--store", type=Path, default=Path("store"))
    ap.add_argument("--keys", type=Path, default=Path("keys"))
    ap.add_argument("--dashboard-token", default=None,
                    help="token for dashboard API access. Generated at random "
                         "if not given. Pass an empty string to disable auth.")
    ap.add_argument("--keep-sim", action="store_true",
                    help="do NOT stop the simulation when this Cloud stops. "
                         "By default Ctrl-C here, and QUITTER on the "
                         "dashboard, also take Gazebo, RViz and the robot "
                         "node down (see agri.session).")
    ap.add_argument("--request", metavar="TARGETS",
                    help="issue one request at startup: ALL, a plant (P2,5, both "
                         "sides), a single station (P2,5R), or a "
                         "comma-separated list. Then keep running.")
    args = ap.parse_args(argv)

    auth_token = args.dashboard_token
    if auth_token is None:
        auth_token = secrets.token_urlsafe(16)

    cloud = Cloud(Store(args.store), args.keys)
    client = mqtt_client("agri-cloud")

    def publish(topic: str, payload: str) -> None:
        client.publish(topic, payload, qos=QOS)

    def on_connect(cl, _u, _f, rc, *_):
        if rc != 0:
            print(f"cloud: broker refused the connection (rc={rc})")
            return
        for topic in (TOPIC_REPORT_ALL, TOPIC_ACK_ALL, TOPIC_STATUS_ALL,
                      TOPIC_REPLY_ALL):
            cl.subscribe(topic, qos=QOS)
        print(f"cloud: connected to {args.broker}:{args.broker_port}, "
              f"listening on {TOPIC_REPORT_ALL}")
        if args.request:
            rid, signed = cloud.build_request(
                args.request if args.request.upper() == ALL
                else [t for t in args.request.split(",") if t.strip()])
            publish(TOPIC_REQUEST, json.dumps(signed))
            print(f"cloud: request {rid} -> {len(cloud.progress[rid]['targets'])}"
                  " station(s)")

    def on_message(_cl, _u, msg):
        nid = node_id_from_topic(msg.topic) or "?"
        if msg.topic.startswith("agri/v1/report/"):
            verdict = cloud.on_report(msg.payload)
            print("cloud:", verdict)
            # Always acknowledge, in both modes. "Il faut toujours des
            # acquittements": a node that never hears that its reading
            # landed cannot tell a filed report from a lost one.
            if not verdict.startswith("REJECTED"):
                publish(topic_query(nid),
                        json.dumps(make_query(STEP_FILED, nid,
                                              label=verdict.split()[0])))
        elif msg.topic.startswith("agri/v1/ack/"):
            cloud.on_ack(msg.payload)
        elif msg.topic.startswith("agri/v1/status/"):
            cloud.on_status(msg.payload, nid)
        elif msg.topic.startswith("agri/v1/reply/"):
            # on_reply DECIDES and returns; publishing is done here, so the
            # whole negotiation is testable with no broker in the room.
            for q in cloud.on_reply(msg.payload, nid):
                publish(topic_query(nid), json.dumps(q))

    client.on_connect, client.on_message = on_connect, on_message
    try:
        client.connect(args.broker, args.broker_port, keepalive=30)
    except OSError as exc:
        print(f"cloud: cannot reach the broker at {args.broker}:"
              f"{args.broker_port} ({exc}).\n"
              "       Start one:  mosquitto -p 1883 -v")
        return 1
    client.loop_start()

    # ONE teardown, reached from two places: Ctrl-C in the console, and the
    # dashboard's STOP button. Guarded because both can happen at once (the
    # button fires, the operator then hits Ctrl-C), and exporting the CSV
    # twice from two threads truncates the file the first one is writing.
    done = threading.Lock()

    def teardown() -> None:
        if not done.acquire(blocking=False):
            return
        print("cloud: stopping")
        try:
            print(f"cloud: exported {cloud.store.export_csv()}")
            print(f"cloud: exported {cloud.store.export_plants_csv()}")
            print(f"cloud: exported {cloud.export_requests_csv()}")
        except Exception as exc:                     # noqa: BLE001
            print(f"cloud: could not export ({exc})")
        client.loop_stop()
        client.disconnect()
        # Take the simulation down with us, unless told not to. Stopping the
        # Cloud and leaving Gazebo, RViz, the bridge and the robot node
        # running is the state that fills a laptop with orphans over an
        # afternoon of demonstrations -- and the operator who pressed QUITTER
        # meant "stop", not "stop half of it". See agri.session for why this
        # is a file and not a signal.
        if args.keep_sim:
            print("cloud: --keep-sim, so the simulation is left running")
        else:
            where = session.ask_simulation_to_stop()
            print(f"cloud: asked the simulation to stop ({where})"
                  if where else
                  "cloud: could not write the stop file; if a simulation is "
                  "running, stop it with Ctrl-C in its own window")

    def shutdown_from_web() -> None:
        # os._exit rather than sys.exit: the console owns the main thread and
        # is blocked inside input(), which no exception raised on an HTTP
        # worker thread can interrupt. Everything worth keeping is already on
        # disk by then -- teardown() has written the CSV and closed MQTT.
        #
        # On its own thread, after a beat, so the handler can return and the
        # reply can leave the socket first. Exiting from inside the handler
        # resets the connection often enough to matter, and the browser then
        # reports a failure for a shutdown that in fact worked.
        teardown()

        def bye() -> None:
            time.sleep(0.3)
            os._exit(0)

        threading.Thread(target=bye, daemon=True).start()

    addr = _my_address()
    base_url = f"http://{addr}:{args.http_port}"
    httpd = ThreadingHTTPServer(
        ("", args.http_port),
        partial(Handler, cloud, publish, auth_token, shutdown_from_web))
    if auth_token:
        print(f"cloud: dashboard on {base_url}?token={auth_token}")
        print(f"       (the token protects the API; pass --dashboard-token '' "
              "to disable)")
    else:
        print(f"cloud: dashboard on {base_url}")
        print("       WARNING: no dashboard token — anyone on the network "
              "can issue requests")
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
        teardown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
