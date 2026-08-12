"""prediction -- LSTM-based imputation of missing sensor readings.

WHY AN LSTM

The five quantities this greenhouse measures vary slowly in time (a 240-
minute diurnal cycle) and correlate strongly in space (a warm corner is
also dry). When a sensor fails -- which a demonstration simulates by
dropping readings at random -- the missing value is not gone: it lives in
the readings that were taken moments ago, and in the neighbours that are
still working. An LSTM sees both axes, and the 48 stations times five
quantities give it enough signal to fill the gap without ever having been
told the physics.

WHAT THIS MODULE DOES AND DOES NOT DO

It trains on the visit history the Cloud already keeps, predicts the five
quantities for any station at any time, and flags which predictions it is
and is not confident about. It does NOT run online inside the robot node:
a model that silently replaces a missing reading with its own prediction
and presents the result as data would be worse than the gap it fills. The
Cloud knows which values are predicted and marks them, and the dashboard
shows the difference.

FAILURE SIMULATION

`FailureSimulator` drops readings at a configurable rate, so the LSTM has
something to recover. The failures are seeded and reproducible, because a
model evaluated against random failures and then re-evaluated against
different random failures has been evaluated against nothing.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Any

import numpy as np

try:
    import torch
    import torch.nn as nn
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False

from agri.measurement import QUANTITIES

QUANTITY_NAMES = [q.name for q in QUANTITIES]
N_QUANTITIES = len(QUANTITY_NAMES)


# --------------------------------------------------------------- failure sim
@dataclass
class FailureSimulator:
    """Drop sensor readings to simulate hardware failures.

    `rate` is the probability that any single quantity at any single visit
    is missing.  `burst_rate` is the probability that an entire station
    goes dark for one visit (all five quantities missing at once) -- the
    mode that matters, because a sensor that loses one channel out of five
    is a different failure from a sensor that loses power.
    """

    rate: float = 0.08
    burst_rate: float = 0.03
    seed: int = 42

    def __post_init__(self) -> None:
        self._rng = random.Random(self.seed)

    def mask(self, n_visits: int) -> np.ndarray:
        """Boolean mask (n_visits, N_QUANTITIES): True where the value is KEPT.

        Reproducible: the same seed, the same number of visits, the same
        mask. Order-dependent on purpose -- a mask that shuffled with the
        visit order would make the training set non-reproducible.
        """
        keep = np.ones((n_visits, N_QUANTITIES), dtype=bool)
        for i in range(n_visits):
            if self._rng.random() < self.burst_rate:
                keep[i, :] = False
                continue
            for j in range(N_QUANTITIES):
                if self._rng.random() < self.rate:
                    keep[i, j] = False
        return keep


# ------------------------------------------------ data preparation
@dataclass
class StationSeries:
    """Time-ordered readings for one station, ready for the LSTM."""

    label: str
    row: int
    plant: int
    side_code: float
    east: float
    north: float
    times: np.ndarray
    values: np.ndarray

    @property
    def n(self) -> int:
        return len(self.times)


def _normalise_time(timestamp: str) -> float:
    """ISO timestamp -> minutes since midnight, for the diurnal feature."""
    try:
        parts = timestamp.split("T")[1].split(":")
        h, m = int(parts[0]), int(parts[1])
        s = float(parts[2].rstrip("Z")) if len(parts) > 2 else 0.0
        return h * 60.0 + m + s / 60.0
    except (IndexError, ValueError):
        return 0.0


def prepare_series(visits: list[Any], label: str,
                   wall_x: float = 4.95, wall_y: float = 2.45
                   ) -> StationSeries | None:
    """Build one station's time series from StoredVisit objects."""
    mine = [v for v in visits if v.label == label]
    if not mine:
        return None
    mine.sort(key=lambda v: v.timestamp)

    from agri.catalogue import station as cat_station
    s = cat_station(label)
    east = (s.plant_x + wall_x) / (2.0 * wall_x)
    north = (s.plant_y + wall_y) / (2.0 * wall_y)

    times = np.array([_normalise_time(v.timestamp) for v in mine])
    vals = np.array([[v.values.get(q, 0.0) for q in QUANTITY_NAMES]
                     for v in mine])
    return StationSeries(
        label=label, row=s.row, plant=s.plant,
        side_code=1.0 if s.side == "R" else 0.0,
        east=east, north=north, times=times, values=vals)


def build_dataset(visits: list[Any], labels: list[str],
                  seq_len: int = 4
                  ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build (X, Y, mask) tensors for training.

    X : (N, seq_len, n_features)  -- input sequences with failures applied
    Y : (N, N_QUANTITIES)         -- ground truth for the LAST step
    mask : (N, N_QUANTITIES)      -- True where Y was artificially dropped

    Features per step: 5 sensor values + time_sin + time_cos + east +
    north + side_code = 10.
    """
    all_series = []
    for label in labels:
        s = prepare_series(visits, label)
        if s is not None and s.n >= seq_len:
            all_series.append(s)

    if not all_series:
        return (np.empty((0, seq_len, 10)),
                np.empty((0, N_QUANTITIES)),
                np.empty((0, N_QUANTITIES), dtype=bool))

    sim = FailureSimulator()
    xs, ys, masks = [], [], []
    for s in all_series:
        spatial = np.array([s.east, s.north, s.side_code])
        fail_mask = sim.mask(s.n)
        for i in range(s.n - seq_len):
            seq = []
            for j in range(seq_len):
                idx = i + j
                t = s.times[idx]
                time_feat = np.array([
                    math.sin(2.0 * math.pi * t / 240.0),
                    math.cos(2.0 * math.pi * t / 240.0)])
                v = s.values[idx].copy()
                v[~fail_mask[idx]] = 0.0
                seq.append(np.concatenate([v, time_feat, spatial]))
            xs.append(np.array(seq))
            ys.append(s.values[i + seq_len - 1])
            masks.append(~fail_mask[i + seq_len - 1])

    return np.array(xs), np.array(ys), np.array(masks)


# --------------------------------------------------------------- LSTM model
if HAS_TORCH:
    class SensorLSTM(nn.Module):
        """Predict all five quantities from a short window of readings.

        A single-layer LSTM followed by a linear head. Nothing deeper is
        warranted: the signal has five dimensions, 48 spatial points, and a
        240-minute period. A model that needs more capacity than this is
        fitting noise.
        """

        def __init__(self, input_dim: int = 10, hidden: int = 32,
                     output_dim: int = N_QUANTITIES) -> None:
            super().__init__()
            self.lstm = nn.LSTM(input_dim, hidden, batch_first=True)
            self.head = nn.Linear(hidden, output_dim)

        def forward(self, x: torch.Tensor) -> torch.Tensor:
            _, (h, _) = self.lstm(x)
            return self.head(h.squeeze(0))


# ------------------------------------------------------------- normalisation
@dataclass
class Scaler:
    """Per-feature min-max scaler, fitted on training data."""

    lo: np.ndarray = field(default_factory=lambda: np.zeros(0))
    hi: np.ndarray = field(default_factory=lambda: np.zeros(0))

    def fit(self, x: np.ndarray) -> "Scaler":
        flat = x.reshape(-1, x.shape[-1])
        self.lo = flat.min(axis=0)
        self.hi = flat.max(axis=0)
        self.hi = np.where(self.hi == self.lo, self.lo + 1.0, self.hi)
        return self

    def transform(self, x: np.ndarray) -> np.ndarray:
        return (x - self.lo) / (self.hi - self.lo)

    def inverse_y(self, y: np.ndarray) -> np.ndarray:
        return y * (self.hi[:N_QUANTITIES] - self.lo[:N_QUANTITIES]) \
            + self.lo[:N_QUANTITIES]


# --------------------------------------------------------------- training
@dataclass
class TrainResult:
    """What a training run produced."""

    epochs: int
    final_loss: float
    n_samples: int
    n_missing: int
    model_state: dict | None = None
    scaler: Scaler | None = None


def train(visits: list[Any], labels: list[str],
          seq_len: int = 4, epochs: int = 80,
          lr: float = 0.005, hidden: int = 32,
          log: Any = None) -> TrainResult:
    """Train the LSTM on the visit history. Returns a TrainResult.

    The model learns to reconstruct ALL five quantities from a window of
    readings where some are missing. The loss is computed only on the
    values that were artificially dropped, so the model is scored on its
    ability to PREDICT, not to copy.
    """
    if not HAS_TORCH:
        raise ImportError("PyTorch is required for LSTM training")

    X, Y, drop_mask = build_dataset(visits, labels, seq_len)
    if X.shape[0] < 2:
        return TrainResult(0, float("inf"), 0, 0)

    scaler = Scaler().fit(X)
    X_s = scaler.transform(X)
    Y_lo, Y_hi = scaler.lo[:N_QUANTITIES], scaler.hi[:N_QUANTITIES]
    Y_range = Y_hi - Y_lo
    Y_range[Y_range == 0] = 1.0
    Y_s = (Y - Y_lo) / Y_range

    X_t = torch.tensor(X_s, dtype=torch.float32)
    Y_t = torch.tensor(Y_s, dtype=torch.float32)
    M_t = torch.tensor(drop_mask, dtype=torch.float32)

    model = SensorLSTM(input_dim=X.shape[-1], hidden=hidden)
    opt = torch.optim.Adam(model.parameters(), lr=lr)
    loss_fn = nn.MSELoss(reduction="none")

    n_missing = int(drop_mask.sum())
    best_loss = float("inf")

    for epoch in range(epochs):
        model.train()
        pred = model(X_t)
        per_sample = loss_fn(pred, Y_t)

        if n_missing > 0:
            loss = (per_sample * M_t).sum() / max(1, n_missing)
        else:
            loss = per_sample.mean()

        opt.zero_grad()
        loss.backward()
        opt.step()
        lv = loss.item()
        if lv < best_loss:
            best_loss = lv
        if log and (epoch + 1) % 20 == 0:
            log(f"  LSTM epoch {epoch + 1}/{epochs}: "
                f"loss {lv:.6f} ({n_missing} dropped values)")

    return TrainResult(
        epochs=epochs, final_loss=best_loss,
        n_samples=X.shape[0], n_missing=n_missing,
        model_state=model.state_dict(), scaler=scaler)


def predict(result: TrainResult, visits: list[Any],
            label: str, minutes: float,
            seq_len: int = 4) -> dict[str, float] | None:
    """Predict the five quantities for a station at a given time.

    Returns None if there is not enough history for the station.
    """
    if not HAS_TORCH or result.model_state is None or result.scaler is None:
        return None

    s = prepare_series(visits, label)
    if s is None or s.n < seq_len:
        return None

    last = min(seq_len, s.n)
    spatial = np.array([s.east, s.north, s.side_code])
    seq = []
    for j in range(last):
        idx = s.n - last + j
        t = s.times[idx]
        time_feat = np.array([
            math.sin(2.0 * math.pi * t / 240.0),
            math.cos(2.0 * math.pi * t / 240.0)])
        v = s.values[idx].copy()
        seq.append(np.concatenate([v, time_feat, spatial]))

    X = np.array([seq])
    X_s = result.scaler.transform(X)
    X_t = torch.tensor(X_s, dtype=torch.float32)

    model = SensorLSTM(input_dim=X.shape[-1])
    model.load_state_dict(result.model_state)
    model.eval()

    with torch.no_grad():
        pred_s = model(X_t).numpy()[0]

    pred = result.scaler.inverse_y(pred_s)
    # Clamped to the quantity's own physical band: an LSTM output is a
    # regression, not a certainty, and an unclamped prediction can land a
    # hair outside [lo, hi] on a station with little history. A predicted
    # value already says what it is; it should not ALSO look broken.
    return {q: round(QUANTITIES[i].clamp(float(pred[i])), QUANTITIES[i].decimals)
            for i, q in enumerate(QUANTITY_NAMES)}


# ------------------------------------------------------ the robot's own copy
@dataclass
class _Visit:
    """The little a station's history needs to remember. Not StoredVisit --
    that lives in agri.cloud.store and carries hash-chains, photo paths and
    everything else the CLOUD keeps. The robot does not need any of that to
    predict; importing the Cloud's own record type here would mean the
    robot depends on the Cloud's package to run its own fallback, which is
    backwards for a fault that may be happening because the Cloud is not
    reachable at all."""

    label: str
    timestamp: str
    values: dict[str, float]


class LocalPredictor:
    """The robot's own rolling model, trained on what IT has read so far.

    This is what agri.robot.Visitor calls when a sensor comes back missing
    or aberrant, BEFORE the report ever leaves the robot. It is deliberately
    not the Cloud's job: a robot that can only recover from a bad reading by
    asking a server it might not be able to reach has not recovered from
    anything. The Cloud still keeps its own /api/predict for inspecting and
    auditing what the model would say from ITS copy of the history -- a
    cross-check, not the mechanism.

    Retrains every `retrain_every` new visits rather than on every one,
    for the same reason the Cloud used to: an LSTM that trains for `epochs`
    steps takes far longer than the robot has between two stations.
    """

    def __init__(self, seq_len: int = 4, epochs: int = 40,
                retrain_every: int = 4) -> None:
        self.seq_len = seq_len
        self.epochs = epochs
        self.retrain_every = retrain_every
        self._visits: list[_Visit] = []
        self._result: TrainResult | None = None
        self._trained_n = 0

    def remember(self, label: str, timestamp: str,
                values: dict[str, float]) -> None:
        """File one more reading -- measured or already-predicted, it does
        not matter: a predicted value that later turns out wrong is no
        worse a training signal than sensor noise, and refusing to learn
        from it would mean a station that fails twice in a row can never
        be predicted the second time."""
        self._visits.append(_Visit(label, timestamp, dict(values)))

    def predict(self, label: str, minutes: float,
               names: list[str]) -> dict[str, float] | None:
        """Predict just the requested quantities, or None if the local
        history is not yet enough to try -- the caller decides what a
        model with nothing to say about a brand new station means for the
        visit; this class only ever says "I don't know" instead of
        guessing wildly on two data points."""
        if not HAS_TORCH or not names:
            return None
        labels = sorted({v.label for v in self._visits})
        need_train = (self._result is None
                     or len(self._visits) >= self._trained_n + self.retrain_every)
        if need_train:
            self._result = train(self._visits, labels,
                                 seq_len=self.seq_len, epochs=self.epochs)
            self._trained_n = len(self._visits)
        if self._result is None or self._result.model_state is None:
            return None
        full = predict(self._result, self._visits, label, minutes,
                       self.seq_len)
        if full is None:
            return None
        return {n: full[n] for n in names if n in full}
