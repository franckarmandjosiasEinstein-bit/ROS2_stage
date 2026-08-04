"""sensors -- the five quantities, synthesised, because Gazebo has no soil.

WHY THIS FILE EXISTS AND WHAT IT IS HONEST ABOUT

Gazebo simulates rigid bodies, light for a camera, and range sensors. It does
not simulate humidity, CO2 concentration or substrate pH, and no plugin makes
it. Those readings have to come from somewhere, and there are two ways to do
it.

The dishonest way is `random.uniform(50, 80)` at the point of use. It looks
like data, it passes through the whole pipeline, and it is worthless: every
visit to the same plant returns an unrelated number, so nothing downstream --
not the anomaly detection, not the time series, not the map -- can ever mean
anything. Worse, it hides bugs: a robot that visits the wrong station still
returns plausible numbers.

The honest way, and the one here, is a GROUND-TRUTH FIELD. Every plant has a
true state that is a deterministic function of where it is, and a reading is
that truth plus sensor noise. Then:

    - the same plant reads roughly the same twice, so a time series is real;
    - neighbouring plants correlate, so the dashboard map shows structure
      rather than confetti;
    - a robot that parks at the wrong cross returns the WRONG plant's values,
      and the Cloud can catch it;
    - the injected anomalies are known, so "did the system surface them?" is
      a question with an answer.

This is exactly the role berries.yaml played in Phase B: the simulator holds
a truth the software is not allowed to see, and the truth is what the results
are scored against.

THE FIELD

Structure, not noise. Reading the greenhouse as a real one would behave:

    temperature  warmer toward the south wall (+y) and toward the far end
                 (+x), where the ventilation is weakest
    humidity     the mirror of temperature -- warm air holds moisture worse
                 near the leaf, so RH drops where the temperature rises
    luminosity   brightest in the middle of the span and dimmer against the
                 side walls, which shade the outer rows
    co2          accumulates in the corners, depleted where light is high
                 because photosynthesis is consuming it
    ph           a property of the substrate, so nearly constant per plant
                 with a slow spatial trend and very little drift

ANOMALIES are placed at named stations and documented below, so a
demonstration has something to find and a report has something to show.
"""

from __future__ import annotations

import hashlib
import math
import random
from dataclasses import dataclass

from agri.catalogue import WALL_X, WALL_Y, plant_position, station
from agri.labels import normalise
from agri.measurement import BY_NAME, Measurement

#: Known faults, injected into the truth. label -> (quantity, delta).
#: A demonstration that surfaces these has surfaced something real; one that
#: does not has a hole between the sensor and the dashboard.
ANOMALIES: dict[str, tuple[str, float]] = {
    "P2,4R": ("humidity", -22.0),    # a dry patch: blocked dripper
    "P2,4L": ("humidity", -20.0),    # the same plant from the other side
    "P3,7R": ("co2", +480.0),        # dead air in the north-east corner
    "P3,7L": ("co2", +455.0),
    "P1,2R": ("ph", +1.35),          # substrate turning alkaline
    "P1,2L": ("ph", +1.30),
}

#: Standard deviation of the sensor noise, per quantity. Small enough that
#: the structure is visible, large enough that two visits differ -- which is
#: what makes the Cloud's averaging worth anything.
NOISE = {"temperature": 0.18, "humidity": 0.9, "luminosity": 260.0,
         "co2": 9.0, "ph": 0.03}


def _unit(v: float, lo: float, hi: float) -> float:
    """Map v in [lo, hi] onto [0, 1], clamped."""
    return max(0.0, min(1.0, (v - lo) / (hi - lo)))


@dataclass
class GreenhouseField:
    """The truth the robot is not allowed to see.

    `seed` makes a whole run reproducible: the same seed gives the same
    readings at the same stations at the same times, which is what lets a
    result be re-derived a week later when somebody asks where a number came
    from.
    """

    seed: int = 20260804

    # -------------------------------------------------------------- truth
    def truth(self, label: str, minutes: float = 0.0) -> dict[str, float]:
        """The true state at a station's plant, `minutes` into the run."""
        label = normalise(label)
        s = station(label)
        px, py = s.plant_x, s.plant_y

        east = _unit(px, -WALL_X, WALL_X)       # 0 at the west end, 1 at east
        north = _unit(py, -WALL_Y, WALL_Y)      # 0 at the south wall, 1 north
        # Distance from the mid-span, which is where the glass lets most light
        # through; the side walls shade the outer rows.
        centrality = 1.0 - abs(py) / WALL_Y

        # A slow diurnal swing so a time series is not a flat line.
        day = math.sin(2.0 * math.pi * minutes / 240.0)

        temperature = 20.0 + 3.2 * east + 2.4 * north + 1.1 * day
        humidity = 78.0 - 9.0 * east - 7.0 * north - 2.5 * day
        luminosity = 6000.0 + 26000.0 * centrality + 5200.0 * day
        co2 = 430.0 + 190.0 * (1.0 - centrality) + 120.0 * east - 60.0 * day
        # pH is a property of the pot: fixed per plant, no diurnal term.
        ph = 6.30 + 0.55 * self._pot_bias(s.row, s.plant)

        values = {"temperature": temperature, "humidity": humidity,
                  "luminosity": luminosity, "co2": co2, "ph": ph}

        if label in ANOMALIES:
            quantity, delta = ANOMALIES[label]
            values[quantity] += delta
        return values

    def _pot_bias(self, row: int, plant: int) -> float:
        """A per-pot constant in [-1, 1], stable across runs and seeds.

        Hashed rather than drawn from the RNG so it does not depend on the
        order in which stations happen to be visited -- otherwise the same
        plant would have a different pH depending on the route taken to it,
        which is precisely the kind of nonsense this module exists to avoid.
        """
        h = hashlib.sha256(f"pot:{row}:{plant}".encode()).digest()
        return (h[0] / 255.0) * 2.0 - 1.0

    # ------------------------------------------------------------ reading
    def read(self, label: str, minutes: float = 0.0,
             pose: tuple[float, float, float] | None = None) -> Measurement:
        """What the robot's sensors report: truth plus noise, clamped."""
        label = normalise(label)
        values = self.truth(label, minutes)
        # Seeded per (run, station, time bucket): repeatable, but a second
        # visit a minute later legitimately differs.
        rng = random.Random(f"{self.seed}:{label}:{int(minutes * 60) // 5}")
        out = {}
        for name, v in values.items():
            q = BY_NAME[name]
            out[name] = round(q.clamp(rng.gauss(v, NOISE[name])), q.decimals)
        return Measurement(label=label, values=out, pose=pose)


def describe_anomalies() -> list[str]:
    """Human-readable list, for the README and the dashboard's legend."""
    lines = []
    for label, (quantity, delta) in sorted(ANOMALIES.items()):
        unit = BY_NAME[quantity].unit
        lines.append(f"{label}: {quantity} {delta:+.2f} {unit}".rstrip())
    return lines
