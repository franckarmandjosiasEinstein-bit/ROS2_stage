"""fruit_map -- the memory the robot did not have.

WHY THIS EXISTS

Until now the detector published per-frame world positions and NOTHING
accumulated them. The robot had no memory of fruit at all: it reacted to
whatever was in the current image, so a red pixel seen in one frame, once, was
acted upon as if it were a strawberry.

The measured consequence, from truth_monitor: 66 of 111 map estimates matched
no real berry, and the harvest phase spent 39% of its time creeping toward
things that were not fruit and then timing out. Those two numbers are the same
number seen from two sides.

Nothing about the detector can fix this. A single observation is a single
observation however good the classifier is; what is missing is the ability to
DISBELIEVE one. That requires memory, and this is it.

THE RULE: TWO DISTINCT PASSES, WITH A BASELINE

A cluster is admitted as real only when it has been seen

  * in at least `min_passes` different scouting passes, AND
  * from robot positions separated by at least `min_baseline` metres.

Both conditions, and each rejects something the other does not.

The PASS count rejects the persistent impostor. A red irrigation pipe is seen
on every frame of a pass; requiring a second, separate pass does not help
against it by itself -- but combined with the baseline it means the thing must
look like fruit from two genuinely different places at two different times.

The BASELINE rejects the view-dependent artefact. A specular highlight on a wet
leaf, a reflection in the glass, a colour fringe at a particular angle: all of
them move or vanish when the viewpoint moves. Counting observations alone
cannot see this -- a detector at 15 Hz gives thirty "independent" observations
of the same artefact from effectively one place. Thirty of those are one piece
of evidence, not thirty.

WHAT THIS MAP RESOLVES: CLUSTERS, NOT BERRIES

Stated plainly because it bounds everything built on top. Measured fruit
position error is 0.21 m. Berries on one plant sit 0.05 to 0.20 m apart. The
error is therefore LARGER than the spacing, so no amount of fusion can tell two
berries on the same plant apart -- attempting it would split one berry into
several ghosts at every observation.

So the fusion radius is set at cluster scale (0.25 m, comfortably below the
0.9 m plant spacing) and an entry means "there is fruit around here", not
"there is a berry exactly here". That is the right granularity for the mission
anyway: the robot stops at a station and picks everything within reach, so the
station is what it needs to know about.

The map is deliberately ignorant of the detector's confidence. It counts
geometry and viewpoints, nothing else, so it keeps working unchanged when the
colour threshold is replaced by a learned model.
"""

from __future__ import annotations

import math


class Cluster:
    """One place where fruit has been seen, and the evidence for it."""

    __slots__ = ("x", "y", "n", "passes", "views", "first_t", "last_t",
                 "picked")

    def __init__(self, x: float, y: float, pass_id: int,
                 view: tuple[float, float], t: float):
        self.x = float(x)
        self.y = float(y)
        self.n = 1
        self.passes = {int(pass_id)}
        self.views = [(float(view[0]), float(view[1]))]
        self.first_t = float(t)
        self.last_t = float(t)
        self.picked = 0

    # ------------------------------------------------------------------
    def add(self, x: float, y: float, pass_id: int,
            view: tuple[float, float], t: float, max_views: int) -> None:
        # Running mean. Every observation weighs the same on purpose: a
        # confidence-weighted mean would let one very sure wrong detection
        # drag the estimate, and the detector's confidence is exactly the
        # thing this map is built not to trust.
        self.n += 1
        self.x += (x - self.x) / self.n
        self.y += (y - self.y) / self.n
        self.passes.add(int(pass_id))
        # Keep a bounded set of viewpoints. The baseline only needs the
        # extremes, and an unbounded list grows without limit over a shift.
        if len(self.views) < max_views:
            self.views.append((float(view[0]), float(view[1])))
        else:
            self._replace_closest(view)
        self.last_t = float(t)

    def _replace_closest(self, view) -> None:
        """Keep the viewpoints as spread out as possible."""
        best_i, best_d = 0, float("inf")
        for i, v in enumerate(self.views):
            d = min(math.dist(v, w) for j, w in enumerate(self.views) if j != i)
            if d < best_d:
                best_i, best_d = i, d
        # Only swap if the newcomer is further from its nearest neighbour
        # than the most redundant view currently stored.
        newcomer = min(math.dist(view, w) for w in self.views)
        if newcomer > best_d:
            self.views[best_i] = (float(view[0]), float(view[1]))

    @property
    def baseline(self) -> float:
        """Largest separation between any two viewpoints, in metres."""
        if len(self.views) < 2:
            return 0.0
        return max(math.dist(a, b)
                   for i, a in enumerate(self.views)
                   for b in self.views[i + 1:])

    def confirmed(self, min_passes: int, min_baseline: float) -> bool:
        return (len(self.passes) >= min_passes
                and self.baseline >= min_baseline)

    def as_dict(self) -> dict:
        return {"x": round(self.x, 3), "y": round(self.y, 3), "n": self.n,
                "passes": sorted(self.passes),
                "baseline": round(self.baseline, 3),
                "picked": self.picked}


class FruitMap:
    """Persistent, corroborated map of fruit clusters."""

    def __init__(self, fusion_radius=0.25, min_passes=2, min_baseline=0.40,
                 max_views=8, arena_half_x=4.9, arena_half_y=2.4):
        self.fusion_radius = float(fusion_radius)
        self.min_passes = int(min_passes)
        self.min_baseline = float(min_baseline)
        self.max_views = int(max_views)
        self.arena_half_x = float(arena_half_x)
        self.arena_half_y = float(arena_half_y)
        self.clusters: list[Cluster] = []
        self.rejected_outside = 0
        self.observations = 0

    # ------------------------------------------------------------ input
    def observe(self, x: float, y: float, pass_id: int,
                view: tuple[float, float], t: float) -> Cluster | None:
        """Fold one detection in. Returns the cluster it joined, or None.

        A position outside the greenhouse is discarded rather than stored:
        it is wrong by construction whatever produced it, and letting it in
        would put a permanent phantom station in the harvest route.
        """
        if abs(x) > self.arena_half_x or abs(y) > self.arena_half_y:
            self.rejected_outside += 1
            return None
        self.observations += 1
        c = self.nearest(x, y, self.fusion_radius)
        if c is None:
            c = Cluster(x, y, pass_id, view, t)
            self.clusters.append(c)
            return c
        c.add(x, y, pass_id, view, t, self.max_views)
        return c

    # ------------------------------------------------------------ query
    def nearest(self, x: float, y: float,
                within: float | None = None) -> Cluster | None:
        best, best_d = None, float("inf")
        for c in self.clusters:
            d = math.hypot(c.x - x, c.y - y)
            if d < best_d:
                best, best_d = c, d
        if best is None:
            return None
        if within is not None and best_d > within:
            return None
        return best

    def is_confirmed_at(self, x: float, y: float,
                        within: float | None = None) -> bool:
        """Is there CORROBORATED fruit near this position?

        This is the gate the harvest uses. A detection that does not land on a
        confirmed cluster is not worth stopping for, and stopping for it is
        precisely what cost 39% of the harvest phase.
        """
        c = self.nearest(x, y, self.fusion_radius if within is None else within)
        return bool(c and not c.picked
                    and c.confirmed(self.min_passes, self.min_baseline))

    def confirmed(self) -> list[Cluster]:
        return [c for c in self.clusters
                if c.confirmed(self.min_passes, self.min_baseline)]

    def pending(self) -> list[Cluster]:
        return [c for c in self.clusters
                if not c.confirmed(self.min_passes, self.min_baseline)]

    def unpicked(self) -> list[Cluster]:
        return [c for c in self.confirmed() if not c.picked]

    def route_from(self, x: float, y: float) -> list[Cluster]:
        """Confirmed, unpicked clusters, nearest-first from a pose."""
        remaining = self.unpicked()
        tour, cur = [], (x, y)
        while remaining:
            nxt = min(remaining, key=lambda c: math.hypot(c.x - cur[0],
                                                          c.y - cur[1]))
            tour.append(nxt)
            remaining.remove(nxt)
            cur = (nxt.x, nxt.y)
        return tour

    # ----------------------------------------------------------- update
    def mark_picked(self, x: float, y: float, within: float = 0.40) -> bool:
        c = self.nearest(x, y, within)
        if c is None:
            return False
        c.picked += 1
        return True

    # ---------------------------------------------------------- reports
    def summary(self) -> dict:
        conf, pend = self.confirmed(), self.pending()
        return {
            "observations": self.observations,
            "clusters": len(self.clusters),
            "confirmed": len(conf),
            "pending_rejected": len(pend),
            "outside_arena": self.rejected_outside,
            "picked": sum(c.picked for c in conf),
            "rule": (f"seen in >= {self.min_passes} passes AND from viewpoints "
                     f">= {self.min_baseline:.2f} m apart"),
        }

    def report_lines(self) -> list[str]:
        s = self.summary()
        out = ["--- fruit map ---",
               "obs       %d detections -> %d clusters"
               % (s["observations"], s["clusters"]),
               "confirmed %d  (%s)" % (s["confirmed"], s["rule"]),
               "rejected  %d clusters never corroborated, %d outside the arena"
               % (s["pending_rejected"], s["outside_arena"])]
        if s["clusters"]:
            share = 100.0 * s["pending_rejected"] / s["clusters"]
            out.append("          %.0f%% of what the detector proposed was "
                       "thrown away for lack of a second viewpoint" % share)
        return out
