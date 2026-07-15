"""Lidar scan-matching localization (framework-independent).

Ported from the validated Webots `localization.py` (Phase 3). Corrects a
drifting odometry prior by aligning the lidar scan to a known reference
map, using a boundary-seeded likelihood field and a coarse+fine search with
a trust gate. See docs/ARCHITECTURE.md for why the field is seeded from
obstacle *surfaces* and not filled footprints.
"""

from __future__ import annotations

from collections import deque

import numpy as np


class ScanMatcher:
    def __init__(self, ref_grid: np.ndarray, resolution: float, arena_size: float,
                 beam_stride: int = 6, sigma_m: float = 0.15) -> None:
        self.ref_grid = ref_grid
        self.resolution = resolution
        self.arena_size = arena_size
        self.beam_stride = beam_stride
        self.sigma_m = sigma_m
        self._cache = None       # cached (idx, base_angle) for the beam layout
        self._cache_key = None   # (n_beams, fov) the cache was built for
        # Likelihood field: distance (cells) to the nearest obstacle SURFACE.
        self._dist = self._distance_field(self._boundary(ref_grid))

    @staticmethod
    def _boundary(grid: np.ndarray) -> np.ndarray:
        occ = grid > 0
        free_neighbor = np.zeros_like(occ)
        free_neighbor[:-1, :] |= ~occ[1:, :]
        free_neighbor[1:, :] |= ~occ[:-1, :]
        free_neighbor[:, :-1] |= ~occ[:, 1:]
        free_neighbor[:, 1:] |= ~occ[:, :-1]
        return occ & free_neighbor

    @staticmethod
    def _distance_field(seed: np.ndarray) -> np.ndarray:
        rows, cols = seed.shape
        dist = np.full((rows, cols), 1_000_000, dtype=np.int32)
        dq = deque()
        for r, c in np.argwhere(seed):
            dist[r, c] = 0
            dq.append((int(r), int(c)))
        while dq:
            r, c = dq.popleft()
            d = dist[r, c] + 1
            for nr, nc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
                if 0 <= nr < rows and 0 <= nc < cols and dist[nr, nc] > d:
                    dist[nr, nc] = d
                    dq.append((nr, nc))
        return dist

    def _beam_layout(self, n: int, fov: float):
        """Cache subsampled beam indices + local angles (constant per correct())."""
        key = (n, fov)
        if self._cache_key != key:
            idx = np.arange(0, n, self.beam_stride)
            base = fov / 2.0 - idx * (fov / (n - 1))
            self._cache = (idx, base)
            self._cache_key = key
        return self._cache

    def _score(self, x, y, th, ranges, fov, max_range) -> float:
        # Vectorised likelihood-field score (identical maths to the per-beam
        # loop, ~10x faster): project all subsampled endpoints, look up the
        # distance field, sum the Gaussian weights.
        ranges = np.asarray(ranges, dtype=float)
        n = ranges.size
        if n < 2:
            return 0.0
        idx, base = self._beam_layout(n, fov)
        r = ranges[idx]
        a = th + base
        finite = np.isfinite(r) & (r < max_range)
        r_safe = np.where(finite, r, 0.0)
        ex = x + r_safe * np.cos(a)
        ey = y + r_safe * np.sin(a)
        half_size = self.arena_size / 2.0
        col = ((ex + half_size) / self.resolution).astype(np.int64)
        row = ((half_size - ey) / self.resolution).astype(np.int64)
        rows, cols = self._dist.shape
        ok = finite & (col >= 0) & (col < cols) & (row >= 0) & (row < rows)
        if not ok.any():
            return 0.0
        d = self._dist[row[ok], col[ok]].astype(float)
        sig_cells = max(1e-6, self.sigma_m / self.resolution)
        two_sig2 = 2.0 * sig_cells * sig_cells
        return float(np.exp(-(d * d) / two_sig2).sum())

    def _search(self, prior, ranges, fov, max_range, lin_win, lin_step, ang_win, ang_step):
        px, py, pth = prior
        best, best_score = prior, -1.0
        dx = -lin_win
        while dx <= lin_win + 1e-9:
            dy = -lin_win
            while dy <= lin_win + 1e-9:
                dth = -ang_win
                while dth <= ang_win + 1e-9:
                    s = self._score(px + dx, py + dy, pth + dth, ranges, fov, max_range)
                    if s > best_score:
                        best_score, best = s, (px + dx, py + dy, pth + dth)
                    dth += ang_step
                dy += lin_step
            dx += lin_step
        return best, best_score

    def correct(self, prior, ranges, fov, max_range):
        """Coarse+fine search with a trust gate (never returns a worse score)."""
        prior_score = self._score(*prior, ranges, fov, max_range)
        coarse, _ = self._search(prior, ranges, fov, max_range,
                                 lin_win=0.30, lin_step=0.10, ang_win=0.10, ang_step=0.05)
        fine, score = self._search(coarse, ranges, fov, max_range,
                                   lin_win=0.06, lin_step=0.02, ang_win=0.04, ang_step=0.02)
        # Strictly-better gate: keep the prior on ties too, so an uninformative
        # scan (few/no finite beams, all scores equal) cannot drift the pose to
        # a search-window corner with no evidence of improvement.
        if score <= prior_score:
            return prior, prior_score
        return fine, score
