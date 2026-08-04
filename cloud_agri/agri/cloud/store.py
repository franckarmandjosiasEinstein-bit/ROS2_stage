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
"""

from __future__ import annotations

import base64
import csv
import json
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

from agri.labels import all_labels
from agri.measurement import QUANTITIES, Measurement


def _safe(label: str) -> str:
    """P2,5R -> P2_5R. Commas are legal in filenames and a nuisance in every
    shell that will ever touch them."""
    return label.replace(",", "_")


@dataclass
class StoredVisit:
    label: str
    timestamp: str
    robot: str
    request_id: str | None
    values: dict[str, float]
    pose: dict[str, float] | None
    parking_error_m: float | None
    photo_path: str | None
    qr_path: str | None
    flags: list[str] = field(default_factory=list)

    def to_json(self) -> dict[str, Any]:
        return {
            "label": self.label, "timestamp": self.timestamp,
            "robot": self.robot, "request_id": self.request_id,
            "values": self.values, "pose": self.pose,
            "parking_error_m": self.parking_error_m,
            "photo": self.photo_path, "qr": self.qr_path,
            "flags": self.flags,
        }

    @classmethod
    def from_json(cls, d: dict[str, Any]) -> "StoredVisit":
        return cls(label=d["label"], timestamp=d["timestamp"],
                   robot=d.get("robot", "?"), request_id=d.get("request_id"),
                   values=d.get("values", {}), pose=d.get("pose"),
                   parking_error_m=d.get("parking_error_m"),
                   photo_path=d.get("photo"), qr_path=d.get("qr"),
                   flags=d.get("flags", []))


class Store:
    """Append-only measurement store. Safe to call from the MQTT thread and
    read from the HTTP thread: one lock, held only around the file."""

    def __init__(self, root: Path | str = "store") -> None:
        self.root = Path(root)
        self.jsonl = self.root / "measurements.jsonl"
        self._lock = threading.Lock()
        self._visits: list[StoredVisit] = []
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
                    self._visits.append(StoredVisit.from_json(json.loads(line)))
                except Exception as exc:            # noqa: BLE001
                    # One corrupt line must not cost the whole campaign. Say
                    # which, keep the rest: a truncated final line is what a
                    # kill during a write looks like, and it is recoverable.
                    print(f"store: skipping {self.jsonl}:{n} ({exc})")

    # -------------------------------------------------------------- adding
    def add_report(self, report: dict[str, Any]) -> StoredVisit:
        """File an OPENED and validated report (see envelope.open_envelope)."""
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
            parking_error_m=report.get("parking_error_m"),
            photo_path=photo_path, qr_path=qr_path,
            flags=[f"out_of_range:{q}" for q in m.out_of_range()])

        with self._lock:
            if any(v.label == visit.label and v.timestamp == visit.timestamp
                   and v.robot == visit.robot for v in self._visits):
                return visit          # redelivery, not a second visit
            self._visits.append(visit)
            with self.jsonl.open("a") as fh:
                fh.write(json.dumps(visit.to_json(), sort_keys=True) + "\n")
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
    def export_csv(self, path: Path | None = None) -> Path:
        path = Path(path or self.root / "measurements.csv")
        cols = (["label", "row", "plant", "side", "timestamp", "robot",
                 "request_id"]
                + [q.name for q in QUANTITIES]
                + ["pose_x", "pose_y", "parking_error_m", "flags"])
        with path.open("w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(cols)
            for v in sorted(self.all_visits(),
                            key=lambda v: (v.label, v.timestamp)):
                row, plant, side = v.label[1], v.label.split(",")[1][:-1], v.label[-1]
                pose = v.pose or {}
                w.writerow([v.label, row, plant, side, v.timestamp, v.robot,
                            v.request_id]
                           + [v.values.get(q.name, "") for q in QUANTITIES]
                           + [pose.get("x", ""), pose.get("y", ""),
                              v.parking_error_m if v.parking_error_m is not None else "",
                              ";".join(v.flags)])
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
