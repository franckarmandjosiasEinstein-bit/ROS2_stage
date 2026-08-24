"""OnlineScanMatcher -- scan matching against a map the robot is building.

Extends the Webots-validated `ScanMatcher` (likelihood-field, coarse+fine
search, trust gate) with adaptations for online SLAM:

1. EXACT ROS BEAM LAYOUT. The parent assumes Webots' clockwise sweep
   (fov/2 - i*step); a ROS LaserScan sweeps angle_min + i*increment.

2. MEAN SCORE OVER KNOWN CELLS ("corridor lock" fix). With a self-built
   map, beams landing in unmapped territory ahead score 0; a sum therefore
   prefers poses in well-mapped territory behind the robot. The mean over
   mapped-cell beams removes that coverage bias.

3. MATCH QUALITY for the map-update gate (map<->pose feedback fix).
   Integrating scans at a slightly-wrong pose smears surfaces into the map;
   `last_quality` lets slam_node skip map updates when the match is poor.

4. SELF-HIT FILTER + FAST CAPPED DISTANCE FIELD. Beams under
   `min_valid_range` (robot body) are dropped; the distance field uses
   vectorised numpy min-propagation instead of pure-Python BFS.

5. CURVATURE-BASED ADAPTIVE GAINS. Instead of the binary observable_axes
   gate (which either applies a correction or blocks it entirely), the
   score-surface curvature at the prior is measured per axis (X, Y, heading)
   and converted to a continuous gain in [0, 1]. High curvature = the scan
   constrains that axis = high gain. Low curvature = corridor / aperture
   problem = gain near zero, so noise on that axis is suppressed without
   blocking real corrections on the other axes. This is the same principle
   as an EKF's innovation gain, without the full covariance machinery.
"""

from __future__ import annotations

import math

import numpy as np

from youbot_control.lib.scan_matcher import ScanMatcher


class OnlineScanMatcher(ScanMatcher):
    DIST_CAP = 12

    def __init__(self, ref_grid: np.ndarray, known: np.ndarray,
                 resolution: float, arena_size: float,
                 angle_min: float, angle_inc: float,
                 beam_stride: int = 3, sigma_m: float = 0.12,
                 min_valid_range: float = 0.35, min_beams: int = 10,
                 min_gain: float = 0.01) -> None:
        super().__init__(ref_grid, resolution, arena_size, beam_stride, sigma_m)
        self.min_beams = min_beams
        self.min_gain = min_gain
        self.known = known
        self.angle_min = angle_min
        self.angle_inc = angle_inc
        self.min_valid_range = min_valid_range
        self.last_quality = 0.0
        self.last_known = 0
        self.last_axis_gains = (1.0, 1.0, 1.0)

    @staticmethod
    def _distance_field(seed: np.ndarray) -> np.ndarray:
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
            self.last_known = 0
            return 0.0
        known = np.zeros_like(ok)
        known[ok] = self.known[row[ok], col[ok]]
        use = ok & known
        n_known = int(use.sum())
        self.last_known = n_known
        if n_known == 0:
            self.last_quality = 0.0
            return 0.0
        d = self._dist[row[use], col[use]].astype(float)
        sig = max(1e-6, self.sigma_m / self.resolution)
        total = float(np.exp(-(d * d) / (2.0 * sig * sig)).sum())
        self.last_quality = total / n_known
        return total / max(n_known, self.min_beams)

    def _axis_gains(self, prior, ranges, max_range,
                    lin_probe=0.06, ang_probe=0.03,
                    curvature_ref=20.0):
        """Per-axis confidence from the score-surface curvature at the prior.

        For each axis (X, Y, heading), measure how sharply the score drops
        when the pose is displaced by +/-probe. High curvature means the
        scan constrains that axis (cross-aisle, heading); low curvature
        means it does not (along-aisle aperture problem).

        Returns (gx, gy, gth) in [0, 1], suitable as per-axis multipliers
        on the matcher's correction.

        curvature_ref normalises the raw curvature into a gain: an axis
        whose curvature equals curvature_ref gets gain 1.0. Bench data:
        cross-aisle curvature ~0.09 / 0.06^2 = 25, along-aisle ~0.003 /
        0.06^2 = 0.8, so 20 separates them well.
        """
        base = self._score(*prior, ranges, 0.0, max_range)
        if self.last_known < self.min_beams:
            return (0.0, 0.0, 0.0)

        gains = []
        for axis in (0, 1):
            plus, minus = list(prior), list(prior)
            plus[axis] += lin_probe
            minus[axis] -= lin_probe
            sp = self._score(*plus, ranges, 0.0, max_range)
            sm = self._score(*minus, ranges, 0.0, max_range)
            curv = abs(sp + sm - 2.0 * base) / (lin_probe * lin_probe)
            gains.append(min(1.0, curv / curvature_ref))

        plus_th = list(prior); plus_th[2] += ang_probe
        minus_th = list(prior); minus_th[2] -= ang_probe
        sp = self._score(*plus_th, ranges, 0.0, max_range)
        sm = self._score(*minus_th, ranges, 0.0, max_range)
        curv_th = abs(sp + sm - 2.0 * base) / (ang_probe * ang_probe)
        gains.append(min(1.0, curv_th / curvature_ref))

        self.last_axis_gains = tuple(gains)
        return self.last_axis_gains

    def correct_online(self, prior, ranges, max_range):
        """Coarse+fine search with curvature-adaptive per-axis gains.

        Returns ((x, y, th), quality, (gx, gy, gth)).
        The per-axis gains tell slam_node how much to trust each component
        of the correction: 1.0 = fully constrained, 0.0 = unobservable.
        """
        prior_score = self._score(*prior, ranges, 0.0, max_range)
        coarse, _ = self._search(prior, ranges, 0.0, max_range,
                                 lin_win=0.08, lin_step=0.04,
                                 ang_win=0.04, ang_step=0.02)
        fine, score = self._search(coarse, ranges, 0.0, max_range,
                                   lin_win=0.03, lin_step=0.01,
                                   ang_win=0.015, ang_step=0.005)

        if score > prior_score + self.min_gain:
            gx, gy, gth = self._axis_gains(prior, ranges, max_range)
            pose = (prior[0] + gx * (fine[0] - prior[0]),
                    prior[1] + gy * (fine[1] - prior[1]),
                    math.atan2(
                        math.sin(prior[2] + gth * math.atan2(
                            math.sin(fine[2] - prior[2]),
                            math.cos(fine[2] - prior[2]))),
                        math.cos(prior[2] + gth * math.atan2(
                            math.sin(fine[2] - prior[2]),
                            math.cos(fine[2] - prior[2])))))
        else:
            pose = prior
            gx, gy, gth = 0.0, 0.0, 0.0

        self._score(*pose, ranges, 0.0, max_range)
        return pose, self.last_quality, (gx, gy, gth)
