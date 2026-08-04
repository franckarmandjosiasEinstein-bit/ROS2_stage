"""measurement -- what the robot reads at a station, and how it is written.

THE QUANTITIES

Five, fixed, in SI or the unit an agronomist actually uses. They are declared
in ONE table with their unit, their plausible range and how many decimals
mean anything, and everything downstream reads that table: the QR encoder,
the range check, the dashboard axes and the CSV header. Adding a sixth sensor
is one line here and nothing anywhere else.

    temperature  degC   air temperature at canopy height
    humidity     %RH    relative humidity
    luminosity   lux    photosynthetically useful light
    co2          ppm    carbon dioxide
    ph           --     substrate pH, dimensionless by definition

WHY THE QR TEXT IS NOT JSON

The brief says the numeric data becomes a QR code. A QR is a thing a human
points a phone at, so the payload should be readable when they do: someone
standing in the greenhouse scanning the code taped to a plant should see the
reading, not a wall of braces. Hence a compact pipe-delimited line that is
both machine-parseable and legible:

    AGRI1|P2,5R|2026-08-04T18:22:31Z|t=21.4|rh=63.2|lux=12480|co2=431|ph=6.42

It is also about a third the size of the equivalent JSON, which matters: QR
capacity is what decides the module count, and a denser code is a code the
robot's own camera can still read back from a photograph.

The leading AGRI1 is a version tag. A reader that meets AGRI2 one day will
say so instead of silently mis-parsing fields that moved.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any

from agri.labels import normalise

QR_MAGIC = "AGRI1"


@dataclass(frozen=True)
class Quantity:
    key: str            # short key used in the QR text
    name: str           # what a human calls it
    unit: str
    lo: float           # plausible range, for the sanity check
    hi: float
    decimals: int

    def clamp(self, v: float) -> float:
        return max(self.lo, min(self.hi, v))

    def format(self, v: float) -> str:
        return f"{v:.{self.decimals}f}" if self.decimals else str(int(round(v)))


#: The table. Order is the order everything displays in.
QUANTITIES: tuple[Quantity, ...] = (
    Quantity("t",   "temperature", "degC", 0.0,    60.0,   1),
    Quantity("rh",  "humidity",    "%RH",  0.0,   100.0,   1),
    Quantity("lux", "luminosity",  "lux",  0.0, 80000.0,   0),
    Quantity("co2", "co2",         "ppm", 250.0,  5000.0,  0),
    Quantity("ph",  "ph",          "",     3.0,    10.0,   2),
)

BY_KEY = {q.key: q for q in QUANTITIES}
BY_NAME = {q.name: q for q in QUANTITIES}


class MeasurementError(ValueError):
    pass


def utc_now() -> str:
    """Timestamps are UTC with a Z, always. A greenhouse in one time zone
    reporting to a cloud in another is the normal case, and 'local time with
    no offset' is how two readings an hour apart end up looking simultaneous."""
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


@dataclass
class Measurement:
    """One visit to one red cross."""

    label: str                       # "P2,5R"
    timestamp: str = field(default_factory=utc_now)
    values: dict[str, float] = field(default_factory=dict)   # by NAME
    #: Where the robot actually stood, in metres. Kept because the difference
    #: between this and the catalogue position is the only honest measure of
    #: how well it parked, and the Cloud shows it.
    pose: tuple[float, float, float] | None = None

    def __post_init__(self) -> None:
        self.label = normalise(self.label)
        missing = [q.name for q in QUANTITIES if q.name not in self.values]
        if missing:
            raise MeasurementError(
                f"{self.label}: no reading for {', '.join(missing)}")

    # ------------------------------------------------------------ QR text
    def to_qr_text(self) -> str:
        parts = [QR_MAGIC, self.label, self.timestamp]
        parts += [f"{q.key}={q.format(self.values[q.name])}"
                  for q in QUANTITIES]
        return "|".join(parts)

    @classmethod
    def from_qr_text(cls, text: str) -> "Measurement":
        parts = (text or "").split("|")
        if len(parts) < 3 + len(QUANTITIES) or parts[0] != QR_MAGIC:
            raise MeasurementError(
                f"not an {QR_MAGIC} payload: {text[:40]!r}")
        label, ts = parts[1], parts[2]
        values: dict[str, float] = {}
        for token in parts[3:]:
            m = re.match(r"^([a-z0-9]+)=(-?[\d.]+)$", token)
            if not m:
                raise MeasurementError(f"cannot read field {token!r}")
            key, raw = m.group(1), m.group(2)
            if key not in BY_KEY:
                raise MeasurementError(f"unknown quantity {key!r}")
            values[BY_KEY[key].name] = float(raw)
        return cls(label=label, timestamp=ts, values=values)

    # --------------------------------------------------------------- JSON
    def to_dict(self) -> dict[str, Any]:
        d: dict[str, Any] = {
            "label": self.label,
            "timestamp": self.timestamp,
            "values": {q.name: self.values[q.name] for q in QUANTITIES},
            "units": {q.name: q.unit for q in QUANTITIES},
        }
        if self.pose is not None:
            d["pose"] = {"x": round(self.pose[0], 3),
                         "y": round(self.pose[1], 3),
                         "yaw_deg": round(math.degrees(self.pose[2]), 1)}
        return d

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "Measurement":
        pose = None
        if isinstance(d.get("pose"), dict):
            p = d["pose"]
            pose = (float(p["x"]), float(p["y"]),
                    math.radians(float(p["yaw_deg"])))
        return cls(label=d["label"], timestamp=d["timestamp"],
                   values={k: float(v) for k, v in d["values"].items()},
                   pose=pose)

    # -------------------------------------------------------------- checks
    def out_of_range(self) -> list[str]:
        """Quantities outside their plausible band.

        NOT an exception: a pH of 9 is alarming, not malformed, and the whole
        point of sending it to a cloud is that somebody sees it. The Cloud
        stores the reading and flags it; refusing it would hide exactly the
        event the system exists to catch.
        """
        return [q.name for q in QUANTITIES
                if not q.lo <= self.values[q.name] <= q.hi]
