"""OnlineScanMatcher -- scan matching against a map the robot is building.

Extends the Webots-validated `ScanMatcher` (likelihood-field, coarse+fine
search, trust gate) with the three changes online SLAM needs. Each one was
isolated on the greenhouse simulation bench (docs in the package README):

1. EXACT ROS BEAM LAYOUT. The parent assumes Webots' clockwise sweep
   (fov/2 - i*step); a ROS LaserScan sweeps angle_min + i*increment. Feeding
   ROS scans through the parent layout mis-pairs ranges with angles.

2. NEUTRAL SCORE FOR UNEXPLORED SPACE ("corridor lock" fix). With a
   self-built map the area AHEAD is unexplored, so a beam landing there
   scores 0 and the best-scoring pose is always a step BEHIND, in fully
   mapped territory -- the matcher then cancels the robot's forward motion
   (error grew at exactly the driving speed on the bench). Unknown-cell
   endpoints now score a neutral 0.5 instead of 0: advancing into the
   unknown is no longer penalised, while known-cell evidence still decides.

3. MATCH QUALITY for the map-update gate (map<->pose feedback fix).
   Integrating scans at a slightly-lagged pose smears surfaces into the map,
   the next matcher rebuild bakes the smear in, and the lag amplifies
   (divergence at ~0.1 m/scan on the bench). `last_quality` (mean weight
   over known-cell beams) lets slam_node skip map updates when the match is
   poor, which broke the feedback loop and took the bench from 1.15 m final
   error to 0.05 m.

4. SELF-HIT FILTER + FAST CAPPED DISTANCE FIELD (real-robot fixes). The
   gpu_lidar sees the robot's own visuals -- arm pedestal (0.07 m, ~50 deg
   wide), camera mast (0.21 m) and basket walls (0.03-0.28 m, most of the
   rear): beams under `min_valid_range` are dropped. And the parent's
   pure-Python BFS distance field took ~1 s on a 200x200 grid, long enough
   for scans to queue up and be matched against FUTURE odometry -- the pose
   lagged the robot at driving speed. The field is now vectorised numpy
   min-propagation capped at 12 cells (weights are ~0 beyond 8 anyway),
   ~50x faster, so the node keeps up with the 10 Hz lidar.
"""

from __future__ import annotations

import numpy as np

from youbot_control.lib.scan_matcher import ScanMatcher


class OnlineScanMatcher(ScanMatcher):
    DIST_CAP = 12                     # cells; exp(-12^2/2sig^2) ~ 0 anyway

    def __init__(self, ref_grid: np.ndarray, known: np.ndarray,
                 resolution: float, arena_size: float,
                 angle_min: float, angle_inc: float,
                 beam_stride: int = 3, sigma_m: float = 0.12,
                 min_valid_range: float = 0.35) -> None:
        super().__init__(ref_grid, resolution, arena_size, beam_stride, sigma_m)
        self.known = known            # bool grid: cell has been observed
        self.angle_min = angle_min
        self.angle_inc = angle_inc
        self.min_valid_range = min_valid_range   # drop lidar self-hits
        self.last_quality = 0.0       # mean weight over known-cell beams

    @staticmethod
    def _distance_field(seed: np.ndarray) -> np.ndarray:
        # Vectorised min-propagation (4-neighbour, capped): ~50x faster than
        # the parent's deque BFS, which stalled the node for ~1 s per rebuild.
        cap = OnlineScanMatcher.DIST_CAP
        dist = np.where(seed, 0, cap).astype(np.int32)
        for _ in range(cap):
            up = np.full_like(dist, cap);    up[:-1, :] = dist[1:, :]
            down = np.full_like(dist, cap);  down[1:, :] = dist[:-1, :]
            left = np.full_like(dist, cap);  left[:, :-1] = dist[:, 1:]
            right = np.full_like(dist, cap); right[:, 1:] = dist[:, :-1]
            best = np.minimum(np.minimum(up, down), np.minimum(left, right)) + 1
            nxt = np.minimum(dist, best)
            if np.array_equal(nxt, dist):
                break
            dist = nxt
        return dist

    def _score(self, x, y, th, ranges, fov, max_range) -> float:
        # `fov` is ignored -- the exact ROS layout replaces the parent's.
        ranges = np.asarray(ranges, dtype=float)
        n = ranges.size
        if n < 2:
            return 0.0
        idx = np.arange(0, n, self.beam_stride)
        r = ranges[idx]
        a = th + self.angle_min + idx * self.angle_inc
        finite = np.isfinite(r) & (r < max_range) & (r > self.min_valid_range)
        r_safe = np.where(finite, r, 0.0)
        ex = x + r_safe * np.cos(a)
        ey = y + r_safe * np.sin(a)
        half = self.arena_size / 2.0
        col = ((ex + half) / self.resolution).astype(np.int64)
        row = ((half - ey) / self.resolution).astype(np.int64)
        rows, cols = self._dist.shape
        ok = finite & (col >= 0) & (col < cols) & (row >= 0) & (row < rows)
        if not ok.any():
            self.last_quality = 0.0
            return 0.0
        known = np.zeros_like(ok)
        known[ok] = self.known[row[ok], col[ok]]
        use = ok & known
        d = self._dist[row[use], col[use]].astype(float)
        sig = max(1e-6, self.sigma_m / self.resolution)
        w = float(np.exp(-(d * d) / (2.0 * sig * sig)).sum())
        self.last_quality = w / max(1, int(use.sum()))
        return w + 0.5 * float((ok & ~known).sum())

    def observable_axes(self, prior, ranges, max_range, probe=0.06, flat=0.01):
        """Which world axes this scan can actually pin down.

        Sliding along a straight corridor does not change what the lidar sees,
        so the score surface is FLAT along the corridor axis: any 'better'
        pose found there is noise, and applying it integrates into drift.
        This probes the curvature of the score around the prior (scores at
        +/-probe versus at the prior) and reports the axes carrying real
        information. Measured in the field: with calibrated odometry the
        estimate ended 1.04 m off truth while the odometry it was fed was
        only 0.13 m off -- the matcher was actively making X worse."""
        base = self._score(*prior, ranges, 0.0, max_range)
        if base <= 0.0:
            return (False, False)
        out = []
        for axis in (0, 1):
            plus, minus = list(prior), list(prior)
            plus[axis] += probe
            minus[axis] -= probe
            curv = (self._score(*plus, ranges, 0.0, max_range)
                    + self._score(*minus, ranges, 0.0, max_range) - 2.0 * base)
            out.append(abs(curv) > flat * abs(base))
        return tuple(out)

    def correct_online(self, prior, ranges, max_range):
        """Coarse+fine search in a window matched to per-scan odometry drift
        (a few cm), NOT the parent's +/-0.30 m: the backward corridor-lock
        jump must stay out of reach, and drift between two scans is tiny.
        Corrections on unobservable axes are dropped (see observable_axes).
        Returns (pose, quality_at_pose)."""
        prior_score = self._score(*prior, ranges, 0.0, max_range)
        coarse, _ = self._search(prior, ranges, 0.0, max_range,
                                 lin_win=0.08, lin_step=0.04,
                                 ang_win=0.04, ang_step=0.02)
        fine, score = self._search(coarse, ranges, 0.0, max_range,
                                   lin_win=0.03, lin_step=0.01,
                                   ang_win=0.015, ang_step=0.005)
        # 1% margin: a correction must clearly beat the prior, otherwise
        # score noise random-walks the pose over many near-tie scans.
        if score > 1.01 * prior_score:
            ox, oy = self.observable_axes(prior, ranges, max_range)
            pose = (fine[0] if ox else prior[0],
                    fine[1] if oy else prior[1],
                    fine[2])
        else:
            pose = prior
        self._score(*pose, ranges, 0.0, max_range)   # refresh last_quality
        return pose, self.last_quality
