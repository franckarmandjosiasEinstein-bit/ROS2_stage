"""store -- where the Cloud keeps what the robot sent.

FORMAT, AND WHY NOT A DATABASE

The brief says "stockage dans un fichier (au format approprié)". Appropriate
here means: openable by a human, appendable without rewriting, and
diff-able. That is JSON Lines -- one measurement per line, append-only --
plus the images written out as real files, plus a CSV rebuilt on demand for
anyone who wants to open the campaign in a spreadsheet.

A database would be defensible and is what a production system would use. It
would also mean the evidence for every number in the report lives inside a
binary file nobody can inspect during a demonstration. For a system whose
whole point is that the data arrived intact, being able to open the file and
read the line is worth more than query speed over a few thousand rows.

    store/
      measurements.jsonl        append-only, one visit per line
      photos/P2,5R/2026....jpg  the photograph, as sent
      qr/P2,5R/2026....png      the QR code, as sent
      measurements.csv          rebuilt on export

WHY THE IMAGES COME BACK OUT OF THE JSON

They arrive base64'd inside the report. Leaving them there would make the
JSONL unreadable -- a 6 kB blob per line -- and would mean the only way to
look at a photograph is to write a program. So the images are written to
disk, the line keeps their paths, and the line stays about 400 bytes and
legible.

DUPLICATES

A request repeated after a timeout produces a second, genuine visit to the
same station: two measurements a minute apart, both real. So the store does
NOT deduplicate by label. It deduplicates by (label, timestamp, robot),
which catches only the case that is actually a duplicate -- the same message
delivered twice, which MQTT at QoS 1 explicitly permits.

THE CHAIN, AND WHAT IT IS HONESTLY WORTH

Every report is verified on arrival: signature, seal, QR against numbers,
pose against catalogue. Then it is written here in plain text, and from
that moment the cryptography has nothing more to say about it. Anyone with
write access to the disk could change a temperature in a filed line, and
nothing in the system would ever notice -- the whole verified pipeline
protects the reading in flight and abandons it on landing.

So each line carries `prev` and `hash`: the SHA-256 of that line's content
together with the hash before it. Change one byte of one reading and its
hash no longer matches, and neither does every hash after it.

What that buys, precisely: a SURGICAL edit becomes detectable, and a
convincing forgery requires rewriting the entire file from the edit point
to the end. What it does not buy: protection from someone who does exactly
that. The chain and the data live on the same disk, so an attacker with
write access can recompute all of it.

It becomes a real control the moment the tip is pinned somewhere the
attacker cannot reach -- an operator noting it at the end of a campaign, a
nightly copy off the machine, a signature made by a key that is not on the
box. `verify_chain()` and `--verify` exist so that pinning the tip is a
thing somebody can actually do, and the export prints it. Saying this
plainly is the point: an integrity control described as more than it is
provides less than nothing.
"""

from __future__ import annotations

import base64
import csv
import hashlib
import json
import threading
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

from agri.catalogue import ROW_Y
from agri.labels import PLANTS_PER_ROW, all_labels
from agri.measurement import QUANTITIES, Measurement, utc_now

#: The one format every timestamp in this system is written in (see
#: measurement.utc_now). Parsed here so a duration can be computed from two
#: of them without guessing.
TIME_FMT = "%Y-%m-%dT%H:%M:%SZ"

#: The two fields that carry the chain. Excluded from the hash of their own
#: line, obviously, and stripped before a visit is reconstructed.
CHAIN_FIELDS = ("prev", "hash")

#: What `prev` says on the first line of a file. A literal rather than an
#: empty string so that a truncated file cannot be mistaken for a fresh one.
GENESIS = "genesis"


def line_hash(record: dict[str, Any], prev: str) -> str:
    """SHA-256 over one line's content and the hash before it.

    Canonical JSON -- sorted keys, no spaces -- because the hash has to be
    reproducible from a file somebody may have reformatted, and because two
    dicts that differ only in key order are the same record.
    """
    body = {k: v for k, v in record.items() if k not in CHAIN_FIELDS}
    raw = json.dumps(body, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(prev.encode() + raw).hexdigest()


def verify_chain(path: Path) -> tuple[int, list[str]]:
    """(lines checked, problems). Empty problems means the file is intact.

    Reports EVERY break rather than stopping at the first: a file that was
    edited in two places should say so, and an operator running this after
    an incident wants the whole picture in one pass.

    A line with no `hash` at all is reported as unchained, not as a break.
    Campaigns filed before the chain existed are legitimate data, and a
    verifier that called them tampering would make its own output useless.
    """
    if not path.exists():
        return 0, []
    problems: list[str] = []
    prev = GENESIS
    checked = unchained = 0
    with path.open() as fh:
        for n, line in enumerate(fh, 1):
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except Exception as exc:                # noqa: BLE001
                problems.append(f"line {n}: not JSON ({exc})")
                continue
            if "hash" not in rec:
                unchained += 1
                # An unchained line still has to advance the chain, or every
                # line after it would be reported as broken.
                prev = GENESIS
                continue
            if rec.get("prev") != prev:
                problems.append(
                    f"line {n} ({rec.get('label', '?')}): follows "
                    f"{str(rec.get('prev'))[:12]}... but the line before it "
                    f"hashes to {prev[:12]}... -- a line was inserted, "
                    f"removed or reordered here")
            want = line_hash(rec, rec.get("prev", ""))
            if rec["hash"] != want:
                problems.append(
                    f"line {n} ({rec.get('label', '?')}): content does not "
                    f"match its own hash -- this line was EDITED after it "
                    f"was filed")
            prev = rec["hash"]
            checked += 1
    if unchained:
        problems.append(
            f"{unchained} line(s) carry no hash: filed before the chain "
            f"existed. Not tampering -- but not covered either.")
    return checked, problems


def _safe(label: str) -> str:
    """P2,5R -> P2_5R. Commas are legal in filenames and a nuisance in every
    shell that will ever touch them."""
    return label.replace(",", "_")


def seconds_between(start: str | None, end: str | None) -> float | None:
    """end - start, in seconds, for two of our own UTC stamps.

    Returns None rather than raising when either is missing or unparseable:
    visits filed before these fields existed have neither, and a campaign
    must stay readable across a change to its own format.
    """
    if not start or not end:
        return None
    try:
        a = datetime.strptime(start, TIME_FMT).replace(tzinfo=timezone.utc)
        b = datetime.strptime(end, TIME_FMT).replace(tzinfo=timezone.utc)
    except ValueError:
        return None
    return round((b - a).total_seconds(), 1)


@dataclass
class StoredVisit:
    label: str
    #: When the ROBOT took the reading, by its own clock. The number the
    #: measurement is about.
    timestamp: str
    robot: str
    request_id: str | None
    values: dict[str, float]
    pose: dict[str, float] | None
    #: How fast the robot was going when it read. Stored because the brief
    #: asks the robot to send it, and because it is the only field that can
    #: tell you a reading was taken on the move.
    velocity: dict[str, float] | None
    parking_error_m: float | None
    photo_path: str | None
    qr_path: str | None
    flags: list[str] = field(default_factory=list)
    #: Quantities the ROBOT itself predicted rather than measured -- see
    #: agri.measurement.Measurement.predicted. Carried through unchanged
    #: from the report; the Cloud does not decide this, it only shows it.
    predicted: list[str] = field(default_factory=list)
    #: THE OTHER TWO CLOCKS. A reading has three interesting moments and the
    #: store used to keep one of them, so "how long did that take" could not
    #: be answered from the file at all:
    #:
    #:   request_issued_at   the Cloud signed the order        (Cloud clock)
    #:   timestamp           the robot read the station        (robot clock)
    #:   received_at         the Cloud opened and filed it     (Cloud clock)
    #:
    #: The first and last are the same clock, so their difference is a real
    #: duration -- end to end, order to evidence -- and not an artefact of
    #: two machines disagreeing about the time.
    request_issued_at: str | None = None
    received_at: str | None = None

    @property
    def latency_s(self) -> float | None:
        """Order to evidence, in seconds. Both stamps are the Cloud's."""
        return seconds_between(self.request_issued_at, self.received_at)

    def to_json(self) -> dict[str, Any]:
        return {
            "label": self.label, "timestamp": self.timestamp,
            "robot": self.robot, "request_id": self.request_id,
            "values": self.values, "pose": self.pose,
            "velocity": self.velocity,
            "parking_error_m": self.parking_error_m,
            "photo": self.photo_path, "qr": self.qr_path,
            "flags": self.flags, "predicted": self.predicted,
            "request_issued_at": self.request_issued_at,
            "received_at": self.received_at,
        }

    @classmethod
    def from_json(cls, d: dict[str, Any]) -> "StoredVisit":
        return cls(label=d["label"], timestamp=d["timestamp"],
                   robot=d.get("robot", "?"), request_id=d.get("request_id"),
                   values=d.get("values", {}), pose=d.get("pose"),
                   velocity=d.get("velocity"),
                   parking_error_m=d.get("parking_error_m"),
                   photo_path=d.get("photo"), qr_path=d.get("qr"),
                   flags=d.get("flags", []), predicted=d.get("predicted", []),
                   request_issued_at=d.get("request_issued_at"),
                   received_at=d.get("received_at"))


class Store:
    """Append-only measurement store. Safe to call from the MQTT thread and
    read from the HTTP thread: one lock, held only around the file."""

    def __init__(self, root: Path | str = "store") -> None:
        self.root = Path(root)
        self.jsonl = self.root / "measurements.jsonl"
        self._lock = threading.Lock()
        self._visits: list[StoredVisit] = []
        #: Hash of the last line written, so an append can chain to it
        #: without re-reading the file.
        self._tip = GENESIS
        self.root.mkdir(parents=True, exist_ok=True)
        (self.root / "photos").mkdir(exist_ok=True)
        (self.root / "qr").mkdir(exist_ok=True)
        self._load()

    # ------------------------------------------------------------- loading
    def _load(self) -> None:
        if not self.jsonl.exists():
            return
        with self.jsonl.open() as fh:
            for n, line in enumerate(fh, 1):
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                    self._visits.append(StoredVisit.from_json(rec))
                    self._tip = rec.get("hash", GENESIS)
                except Exception as exc:            # noqa: BLE001
                    # One corrupt line must not cost the whole campaign. Say
                    # which, keep the rest: a truncated final line is what a
                    # kill during a write looks like, and it is recoverable.
                    print(f"store: skipping {self.jsonl}:{n} ({exc})")

        # Say it at startup, once, rather than making somebody remember to
        # run --verify. A store that was edited between two runs is exactly
        # the thing nobody thinks to check.
        _, problems = verify_chain(self.jsonl)
        for p in problems:
            print(f"store: INTEGRITY {p}")

    def verify(self) -> tuple[int, list[str]]:
        """(lines checked, problems) for this store's own file."""
        return verify_chain(self.jsonl)

    @property
    def tip(self) -> str:
        """The hash of the last filed line.

        Worth recording at the end of a campaign somewhere other than this
        machine. That -- and only that -- is what turns the chain from a
        consistency check into an integrity control.
        """
        return self._tip

    # -------------------------------------------------------------- adding
    def add_report(self, report: dict[str, Any],
                   request_issued_at: str | None = None) -> StoredVisit:
        """File an OPENED and validated report (see envelope.open_envelope).

        `request_issued_at` is the Cloud's own note of when it signed the
        order this reading answers. The store cannot know it -- requests are
        the server's business -- so the caller passes it down, and it is
        optional because a report can legitimately arrive with no request
        behind it (a re-delivery after a restart, a standalone run).
        """
        m = Measurement.from_dict(report["measurement"])
        stamp = m.timestamp.replace(":", "").replace("-", "")

        photo_path = qr_path = None
        photo = report.get("photo") or {}
        if photo.get("data_b64"):
            p = self.root / "photos" / _safe(m.label) / f"{stamp}.jpg"
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_bytes(base64.b64decode(photo["data_b64"]))
            photo_path = str(p.relative_to(self.root))
        if report.get("qr_png_b64"):
            p = self.root / "qr" / _safe(m.label) / f"{stamp}.png"
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_bytes(base64.b64decode(report["qr_png_b64"]))
            qr_path = str(p.relative_to(self.root))

        visit = StoredVisit(
            label=m.label, timestamp=m.timestamp,
            robot=report.get("robot", "?"), request_id=report.get("request_id"),
            values=m.values, pose=report["measurement"].get("pose"),
            velocity=report["measurement"].get("velocity"),
            parking_error_m=report.get("parking_error_m"),
            photo_path=photo_path, qr_path=qr_path,
            flags=[f"out_of_range:{q}" for q in m.out_of_range()],
            predicted=list(m.predicted),
            request_issued_at=request_issued_at, received_at=utc_now())

        with self._lock:
            if any(v.label == visit.label and v.timestamp == visit.timestamp
                   and v.robot == visit.robot for v in self._visits):
                return visit          # redelivery, not a second visit
            self._visits.append(visit)
            rec = visit.to_json()
            rec["prev"] = self._tip
            rec["hash"] = line_hash(rec, self._tip)
            with self.jsonl.open("a") as fh:
                fh.write(json.dumps(rec, sort_keys=True) + "\n")
            # Only after the write. A tip advanced before a failed write
            # would chain the next line to something that is not on disk.
            self._tip = rec["hash"]
        return visit

    # ------------------------------------------------------------- reading
    def all_visits(self) -> list[StoredVisit]:
        with self._lock:
            return list(self._visits)

    def latest_by_label(self) -> dict[str, StoredVisit]:
        """Most recent visit per station. The dashboard's main view."""
        out: dict[str, StoredVisit] = {}
        for v in self.all_visits():
            if v.label not in out or v.timestamp >= out[v.label].timestamp:
                out[v.label] = v
        return out

    def history(self, label: str) -> list[StoredVisit]:
        return sorted((v for v in self.all_visits() if v.label == label),
                      key=lambda v: v.timestamp)

    def coverage(self) -> tuple[int, int]:
        """(stations measured at least once, stations that exist)."""
        return len(self.latest_by_label()), len(all_labels())

    # -------------------------------------------------------------- export
    #: The three clocks, spelled out in the header rather than left to a
    #: column called "timestamp" that could be any of them. Whoever opens
    #: this file in a spreadsheet six months from now is not going to read
    #: store.py to find out which moment they are looking at.
    CSV_COLUMNS = (["label", "row", "plant", "side",
                    "request_id", "request_issued_at", "measured_at",
                    "received_at", "latency_s", "robot"]
                   + [q.name for q in QUANTITIES]
                   + ["pose_x", "pose_y", "parking_error_m", "flags",
                      "predicted"])

    def export_csv(self, path: Path | None = None) -> Path:
        path = Path(path or self.root / "measurements.csv")
        with path.open("w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(self.CSV_COLUMNS)
            for v in sorted(self.all_visits(),
                            key=lambda v: (v.label, v.timestamp)):
                row, plant, side = v.label[1], v.label.split(",")[1][:-1], v.label[-1]
                pose = v.pose or {}
                w.writerow([v.label, row, plant, side,
                            v.request_id or "", v.request_issued_at or "",
                            v.timestamp, v.received_at or "",
                            "" if v.latency_s is None else v.latency_s,
                            v.robot]
                           + [v.values.get(q.name, "") for q in QUANTITIES]
                           + [pose.get("x", ""), pose.get("y", ""),
                              v.parking_error_m if v.parking_error_m is not None else "",
                              ";".join(v.flags), ";".join(v.predicted)])
        return path

    # ------------------------------------------------- per-plant export
    #: ONE ROW PER PLANT, both sides on the line, in the same shape the
    #: dashboard's /table page draws. Two files rather than one because they
    #: answer different questions and neither is a view of the other:
    #:
    #:   measurements.csv   one row per VISIT. The audit trail. A station
    #:                      measured three times has three rows, each with
    #:                      its own three timestamps, and nothing is lost.
    #:   plants.csv         one row per PLANT, latest reading per side. The
    #:                      agronomic view, and the one an operator actually
    #:                      reads, because the comparison that carries the
    #:                      information is R against L on the same plant.
    #:
    #: Collapsing the first into the second would discard the history; not
    #: writing the second means the comparison has to be done by hand,
    #: forty lines apart, in a spreadsheet.
    PLANT_COLUMNS = (["plant", "row", "index"]
                     + [f"R_{c}" for c in
                        ["measured_at", "received_at"]
                        + [q.name for q in QUANTITIES]
                        + ["parking_error_m", "flags", "predicted", "photo", "qr"]]
                     + [f"L_{c}" for c in
                        ["measured_at", "received_at"]
                        + [q.name for q in QUANTITIES]
                        + ["parking_error_m", "flags", "predicted", "photo", "qr"]])

    def export_plants_csv(self, path: Path | None = None) -> Path:
        path = Path(path or self.root / "plants.csv")
        latest = self.latest_by_label()

        def side(row: int, plant: int, s: str) -> list:
            v = latest.get(f"P{row},{plant}{s}")
            if v is None:
                # Empty, not zero: a plant that was never visited on this
                # side has no reading, which is a different fact from a
                # reading of 0.0 and must not export as one.
                return [""] * (5 + len(QUANTITIES) + 2)
            return ([v.timestamp, v.received_at or ""]
                    + [v.values.get(q.name, "") for q in QUANTITIES]
                    + [v.parking_error_m if v.parking_error_m is not None
                       else "",
                       ";".join(v.flags), ";".join(v.predicted),
                       v.photo_path or "", v.qr_path or ""])

        with path.open("w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(self.PLANT_COLUMNS)
            for row in sorted(ROW_Y):
                for plant in range(1, PLANTS_PER_ROW + 1):
                    w.writerow([f"P{row},{plant}", row, plant]
                               + side(row, plant, "R")
                               + side(row, plant, "L"))
        return path


def summarise(visits: Iterable[StoredVisit]) -> dict[str, Any]:
    """Campaign-level numbers, for the dashboard header."""
    visits = list(visits)
    parks = [v.parking_error_m for v in visits if v.parking_error_m is not None]
    return {
        "visits": len(visits),
        "stations": len({v.label for v in visits}),
        "flagged": sum(1 for v in visits if v.flags),
        "worst_parking_m": round(max(parks), 3) if parks else None,
        "mean_parking_m": round(sum(parks) / len(parks), 3) if parks else None,
    }


def main(argv: list[str] | None = None) -> int:
    """`python3 -m agri.cloud.store --verify` -- check a filed campaign.

    Exists so that checking is a command somebody can run, and can be put in
    a cron job, rather than a property the code merely claims. Exit status
    is 0 for an intact chain and 1 for anything else, so it composes.
    """
    import argparse                                      # noqa: PLC0415

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--root", type=Path, default=Path("store"))
    ap.add_argument("--verify", action="store_true",
                    help="check the hash chain and print the tip")
    args = ap.parse_args(argv)

    path = args.root / "measurements.jsonl"
    if not path.exists():
        print(f"{path} does not exist -- nothing filed yet.")
        return 0
    checked, problems = verify_chain(path)
    tip = ""
    with path.open() as fh:
        for line in fh:
            if line.strip():
                try:
                    tip = json.loads(line).get("hash", "")
                except Exception:                        # noqa: BLE001, S110
                    pass
    print(f"{path}: {checked} chained line(s)")
    if problems:
        for p in problems:
            print(f"  PROBLEM  {p}")
    else:
        print("  the chain is intact")
    if tip:
        print(f"\n  tip  {tip}")
        print("  Record this somewhere other than this machine. The chain\n"
              "  proves the file is self-consistent; only a tip kept out of\n"
              "  reach proves it is the same file you were given.")
    return 1 if problems else 0


if __name__ == "__main__":
    raise SystemExit(main())
