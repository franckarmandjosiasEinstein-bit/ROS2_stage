"""map_eval -- score the map the robot built against the real greenhouse.

The mapping laps are only worth running if there is a way to say whether the
map that came out of them is any good, and "it looks about right in RViz" is
not one. This node reads the true geometry out of the world file and prints,
after every survey lap and periodically in between:

    interior   the wall-to-wall dimensions the map implies, vs the real ones
    rows       where the map puts the three plant gutters, vs where they are
    coverage   how much of the real free floor has actually been observed
    clutter    cells the map calls occupied where reality is open floor

Ground truth is read from the SDF, so it cannot fall out of step with the
world the way a hard-coded constant would. This is a measurement instrument:
it consumes truth and produces numbers for the log. Nothing in the control
stack subscribes to it, and the robot never sees any of it.

Subscribes:  /map_slam (nav_msgs/OccupancyGrid), else /map
             /survey_lap (std_msgs/Int32)
"""

from __future__ import annotations

import math
import os
import re

import numpy as np
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int32


class MapEval(Node):
    def __init__(self) -> None:
        super().__init__("map_eval")
        self.declare_parameter("world_file", "")
        self.declare_parameter("report_period", 60.0)
        # A cell counts as observed once the mapper has committed to it either
        # way; the ROS grid carries -1 for unknown, 0 free, 100 occupied.
        self.declare_parameter("wall_margin", 0.25)   # m: band around a true
                                                      # wall that a mapped wall
                                                      # cell may fall in
        self._grid = None
        self._source = None
        self._walls, self._rows = self._load_world()
        # Preference order, best evidence first. /map_slam is the SLAM belief;
        # /map_raw is the mapper's log-odds evidence; /map is the PLANNING grid,
        # inflated by the robot radius -- that fattens every wall by ~0.36 m and
        # reports the greenhouse ~20 cm short with 15% phantom clutter, which is
        # exactly what the last run measured because it was the only topic
        # available with SLAM off. Never fall back once a better one appears.
        self.create_subscription(
            OccupancyGrid, "map_slam", lambda m: self._on_map(m, "map_slam"), 1)
        self.create_subscription(
            OccupancyGrid, "map_raw", lambda m: self._on_map(m, "map_raw"), 1)
        self.create_subscription(
            OccupancyGrid, "map", lambda m: self._on_map(m, "map (inflated)"), 1)
        self.create_subscription(Int32, "survey_lap", self._on_lap, 5)
        self.create_timer(float(self.get_parameter("report_period").value),
                          lambda: self._report("periodic"))
        if self._walls:
            self.get_logger().info(
                "map_eval up: reference interior %.02f x %.02f m, %d plant rows"
                % (self._walls[1] - self._walls[0], self._walls[3] - self._walls[2],
                   len(self._rows)))
        else:
            self.get_logger().warn(
                "map_eval up but the world file could not be read -- no reference.")

    # ------------------------------------------------------------- reference
    def _load_world(self):
        """Interior extent (x0, x1, y0, y1) and gutter centre lines, from SDF.

        Parsed rather than hard-coded: the moment someone edits the world, a
        constant here would start quietly scoring against a greenhouse that no
        longer exists."""
        path = str(self.get_parameter("world_file").value)
        if not path:
            try:
                path = os.path.join(get_package_share_directory("youbot_gazebo"),
                                    "worlds", "greenhouse.sdf")
            except Exception:
                return None, []
        try:
            text = open(path).read()
        except OSError as exc:
            self.get_logger().warn(f"No world reference ({exc}).")
            return None, []

        models = re.findall(
            r'<model name="([^"]+)">(.*?)</model>', text, flags=re.S)
        walls, rows = {}, []
        for name, body in models:
            pose = re.search(r"<pose>([^<]+)</pose>", body)
            size = re.search(r"<size>([^<]+)</size>", body)
            if not pose or not size:
                continue
            p = [float(v) for v in pose.group(1).split()]
            s = [float(v) for v in size.group(1).split()]
            if name.startswith("wall_"):
                walls[name] = (p[0], p[1], s[0], s[1])
            elif name.startswith("gutter_"):
                rows.append(p[1])
        if len(walls) < 4:
            return None, sorted(rows)
        # Interior faces: each wall's inner side, i.e. its centre pulled in by
        # half its thickness.
        x0 = walls["wall_x_neg"][0] + walls["wall_x_neg"][2] / 2.0
        x1 = walls["wall_x_pos"][0] - walls["wall_x_pos"][2] / 2.0
        y0 = walls["wall_y_neg"][1] + walls["wall_y_neg"][3] / 2.0
        y1 = walls["wall_y_pos"][1] - walls["wall_y_pos"][3] / 2.0
        return (x0, x1, y0, y1), sorted(rows)

    # ---------------------------------------------------------------- inputs
    RANK = {"map_slam": 2, "map_raw": 1, "map (inflated)": 0}

    def _on_map(self, msg: OccupancyGrid, source: str) -> None:
        if self._source is not None and \
                self.RANK[source] < self.RANK[self._source]:
            return
        if self._source != source:
            self._source = source
            self.get_logger().info(f"map_eval scoring /{source}.")
        self._grid = msg

    def _on_lap(self, msg: Int32) -> None:
        self._report(f"after survey lap {int(msg.data)}")

    # --------------------------------------------------------------- scoring
    def _cells(self):
        """(occupied, known, xs, ys) as arrays over the grid, in map metres."""
        g = self._grid
        data = np.asarray(g.data, dtype=np.int16).reshape(g.info.height,
                                                          g.info.width)
        res = g.info.resolution
        x0 = g.info.origin.position.x
        y0 = g.info.origin.position.y
        # Row 0 of a ROS OccupancyGrid is the bottom row (y = origin).
        ys = y0 + (np.arange(g.info.height) + 0.5) * res
        xs = x0 + (np.arange(g.info.width) + 0.5) * res
        return data >= 50, data >= 0, xs, ys

    def _report(self, when: str) -> None:
        if self._grid is None:
            self.get_logger().info("map_eval: no map received yet.", once=True)
            return
        occ, known, xs, ys = self._cells()
        gx, gy = np.meshgrid(xs, ys)
        lines = [f"--- map vs reality ({when}, source /{self._source}) ---"]

        if self._walls is not None:
            tx0, tx1, ty0, ty1 = self._walls
            m = float(self.get_parameter("wall_margin").value)
            # Interior, measured: the extreme occupied cells that sit in the
            # band where a wall really is. Taking the extremes over ALL
            # occupied cells would measure whatever stray cell drifted
            # furthest out, not the building.
            def face(mask, coord, target):
                sel = mask & (np.abs(coord - target) < m)
                return float(np.mean(coord[sel])) if sel.any() else None

            fx0 = face(occ, gx, tx0)
            fx1 = face(occ, gx, tx1)
            fy0 = face(occ, gy, ty0)
            fy1 = face(occ, gy, ty1)
            if None not in (fx0, fx1, fy0, fy1):
                mw, mh = fx1 - fx0, fy1 - fy0
                tw, th = tx1 - tx0, ty1 - ty0
                # A perfect map still reads about one cell short on each axis:
                # a wall cell's CENTRE is up to half a cell inside the face it
                # represents. Quote the grid size so a -6 cm on a 0.05 m grid
                # is read as "exact", which is what it is.
                lines.append(
                    "interior  measured %.02f x %.02f m | real %.02f x %.02f m "
                    "| error %+.0f cm / %+.0f cm (grid %.0f cm, so +/-%.0f cm "
                    "is exact)"
                    % (mw, mh, tw, th, 100 * (mw - tw), 100 * (mh - th),
                       100 * self._grid.info.resolution,
                       100 * self._grid.info.resolution))
            else:
                missing = [n for n, v in (("west", fx0), ("east", fx1),
                                          ("south", fy0), ("north", fy1))
                           if v is None]
                lines.append("interior  incomplete -- no mapped wall on the "
                             + ", ".join(missing) + " side")

            # Coverage over the real free floor (inside the walls, outside the
            # gutter footprints -- the robot cannot observe under a bench).
            # Stand off the walls as well as the gutters: a correctly mapped
            # wall cell sits just inside the interior, and counting it as
            # clutter would score a perfect map at several percent wrong.
            pad = 0.15
            floor = ((gx > tx0 + pad) & (gx < tx1 - pad)
                     & (gy > ty0 + pad) & (gy < ty1 - pad))
            for ry in self._rows:
                floor &= np.abs(gy - ry) > 0.30
            if floor.any():
                lines.append("coverage  %.0f%% of the free floor observed "
                             "(%d of %d cells)"
                             % (100.0 * (known & floor).sum() / floor.sum(),
                                int((known & floor).sum()), int(floor.sum())))
                clutter = (occ & floor).sum()
                lines.append("clutter   %.01f%% of the free floor mapped as "
                             "obstacle (%d cells)"
                             % (100.0 * clutter / floor.sum(), int(clutter)))

        # Plant rows: collapse the occupied cells inside the greenhouse onto y
        # and look for the peaks. Three gutters -> three bands.
        if self._rows and self._walls is not None:
            tx0, tx1, ty0, ty1 = self._walls
            band = occ & (gx > tx0 + 0.4) & (gx < tx1 - 0.4)
            found = []
            for ry in self._rows:
                sel = band & (np.abs(gy - ry) < 0.35)
                found.append(float(np.mean(gy[sel])) if sel.any() else None)
            got = sum(1 for f in found if f is not None)
            lines.append("rows      %d of %d plant rows in the map" % (got, len(self._rows)))
            lines.append("          " + " | ".join(
                "y %+.02f (real %+.02f)" % (f, r) if f is not None
                else "y  ----  (real %+.02f)" % r
                for f, r in zip(found, self._rows)))
        self.get_logger().info("\n".join(lines))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapEval()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass          # RuntimeError: message arriving mid-shutdown (rclpy)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
