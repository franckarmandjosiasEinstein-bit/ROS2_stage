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
"""

from __future__ import annotations

import numpy as np

from youbot_control.lib.scan_matcher import ScanMatcher


class OnlineScanMatcher(ScanMatcher):
    def __init__(self, ref_grid: np.ndarray, known: np.ndarray,
                 resolution: float, arena_size: float,
                 angle_min: float, angle_inc: float,
                 beam_stride: int = 3, sigma_m: float = 0.12) -> None:
        super().__init__(ref_grid, resolution, arena_size, beam_stride, sigma_m)
        self.known = known            # bool grid: cell has been observed
        self.angle_min = angle_min
        self.angle_inc = angle_inc
        self.last_quality = 0.0       # mean weight over known-cell beams

    def _score(self, x, y, th, ranges, fov, max_range) -> float:
        # `fov` is ignored -- the exact ROS layout replaces the parent's.
        ranges = np.asarray(ranges, dtype=float)
        n = ranges.size
        if n < 2:
            return 0.0
        idx = np.arange(0, n, self.beam_stride)
        r = ranges[idx]
        a = th + self.angle_min + idx * self.angle_inc
        finite = np.isfinite(r) & (r < max_range)
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

    def correct_online(self, prior, ranges, max_range):
        """Coarse+fine search in a window matched to per-scan odometry drift
        (a few cm), NOT the parent's +/-0.30 m: the backward corridor-lock
        jump must stay out of reach, and drift between two scans is tiny.
        Returns (pose, quality_at_pose)."""
        prior_score = self._score(*prior, ranges, 0.0, max_range)
        coarse, _ = self._search(prior, ranges, 0.0, max_range,
                                 lin_win=0.08, lin_step=0.04,
                                 ang_win=0.04, ang_step=0.02)
        fine, score = self._search(coarse, ranges, 0.0, max_range,
                                   lin_win=0.03, lin_step=0.01,
                                   ang_win=0.015, ang_step=0.005)
        pose = fine if score > prior_score else prior
        self._score(*pose, ranges, 0.0, max_range)   # refresh last_quality
        return pose, self.last_quality
