"""Stage 4 -- map the real greenhouse with the robot PUSHED, not driven.

Pushing the robot removes two variables at once. The motors are off, so wheel
slip is whatever the pushing produces rather than whatever the controller
produces; and no navigation code runs, so a bad map cannot be blamed on a bad
path and vice versa. If the map is wrong here, the mapper or the lidar is
wrong, and stage 3 already told you which.

The criteria are dimensional, against tape measurements the operator declares
before starting. A map that "looks right" in RViz is not a result: the Phase B
simulation map was accurate to -4 cm / +6 cm and that number is only
meaningful because something compared it to a known truth.

The output of this stage is the map that stages 5 and 6 localise against.
Save it. It is the single most valuable artefact of the whole commissioning.
"""

from __future__ import annotations

import math

import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray

from youbot_commissioning.lib.stage import CommissioningStage, run


class Stage4(CommissioningStage):
    STAGE = 4
    SLUG = "manual_map"
    TITLE = "Manual mapping of the real greenhouse"
    PROCEDURE = """
    BEFORE ARMING
      1. Stage 3 has PASSED, including the glass stations. If glass was
         invisible and you have not marked the panes, STOP: you are about to
         build a map with no walls in it.
      2. Measure the greenhouse interior with a tape: length and width between
         the inner faces. Declare them:
           ros2 topic pub -1 /commissioning/expect \\
             std_msgs/Float32MultiArray "{data: [length_m, width_m]}"
      3. Motors OFF or the base disarmed. You are pushing.

    WHAT WILL HAPPEN
      Nothing, until you push. Walk the robot slowly (well under 0.3 m/s)
      along every aisle and both end corridors, twice. Slower is better: the
      mapper integrates one scan every few ticks and a fast push aliases.
      Watch the map grow in RViz.

    WHEN DONE
      Ctrl-C. The node measures the map's interior extent and compares it with
      your tape. Then SAVE THE MAP:
        ros2 run nav2_map_server map_saver_cli -f serre_reference

    PASS MEANS
      The map's dimensions match the tape within tolerance, coverage is
      complete, and there is no phantom clutter in the aisles. Clutter in the
      aisles means the robot will refuse to plan through them tomorrow.
    """

    def __init__(self):
        super().__init__("stage4_manual_map")
        self.declare_parameter("dimension_tolerance", 0.10)   # m
        self.declare_parameter("min_coverage", 0.95)          # fraction
        self.declare_parameter("max_clutter", 0.02)           # fraction
        self.declare_parameter("map_topic", "map")
        self.declare_parameter("occupied_threshold", 50)

        self._expect = None      # (length_m, width_m)
        self._grid = None
        self._info = None
        self._updates = 0

        self.create_subscription(
            OccupancyGrid, str(self.get_parameter("map_topic").value),
            self._on_map, 1)
        self.create_subscription(Float32MultiArray, "/commissioning/expect",
                                 self._on_expect, 10)

    def _on_expect(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 2:
            return
        self._expect = (float(msg.data[0]), float(msg.data[1]))
        self.get_logger().info(
            f"tape measurements recorded: interior "
            f"{self._expect[0]:.3f} x {self._expect[1]:.3f} m. Start pushing.")

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._info = msg.info
        self._grid = np.asarray(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)
        self._updates += 1
        if self._updates % 20 == 0:
            known = int((self._grid >= 0).sum())
            total = self._grid.size
            self.get_logger().info(
                f"  map {msg.info.width}x{msg.info.height} @ "
                f"{msg.info.resolution:.3f} m, {100.0 * known / total:.1f}% "
                "of cells observed -- keep pushing")

    def stop(self) -> None:
        super().stop()
        if self._grid is not None and not self.report.checks:
            self._conclude()

    def _conclude(self) -> None:
        thr = int(self.get_parameter("occupied_threshold").value)
        res = self._info.resolution
        occupied = self._grid >= thr
        known = self._grid >= 0
        free = known & ~occupied

        self.report.record("map_width_cells", int(self._info.width))
        self.report.record("map_height_cells", int(self._info.height))
        self.report.record("resolution_m", float(res))
        self.report.record("map_updates_seen", self._updates)

        rows = np.where(occupied.any(axis=1))[0]
        cols = np.where(occupied.any(axis=0))[0]
        if rows.size < 2 or cols.size < 2:
            self.report.check("map contains walls", 0, "==", 1,
                              note="no occupied cells: the lidar saw nothing "
                                   "solid. Re-read stage 3.")
            return

        # Outer extent of occupied cells, minus one wall thickness each side,
        # is the interior. The walls themselves occupy cells, so the interior
        # is the gap between the innermost occupied rows/columns.
        measured_len = float((cols[-1] - cols[0]) * res)
        measured_wid = float((rows[-1] - rows[0]) * res)
        self.report.record("measured_extent_m", [measured_len, measured_wid])

        coverage = float(known.sum()) / float(known.size)
        # Clutter: occupied cells that are ISOLATED -- no occupied neighbour in
        # the 8-neighbourhood. A wall is contiguous; a phantom is a speck.
        pad = np.zeros((occupied.shape[0] + 2, occupied.shape[1] + 2), bool)
        pad[1:-1, 1:-1] = occupied
        neighbours = np.zeros_like(occupied, dtype=np.int16)
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                neighbours += pad[1 + dr:1 + dr + occupied.shape[0],
                                  1 + dc:1 + dc + occupied.shape[1]].astype(np.int16)
        isolated = int((occupied & (neighbours == 0)).sum())
        clutter = isolated / max(1, int(occupied.sum()))

        self.report.record("coverage_fraction", coverage)
        self.report.record("clutter_fraction", clutter)
        self.report.record("free_cells", int(free.sum()))
        self.report.record("occupied_cells", int(occupied.sum()))

        tol = float(self.get_parameter("dimension_tolerance").value)
        if self._expect is None:
            self.report.check("tape measurements declared", 0, "==", 1,
                              note="publish /commissioning/expect with the "
                                   "interior length and width before pushing")
        else:
            self.report.check("interior length error",
                              abs(measured_len - self._expect[0]), "<=", tol,
                              "m", f"map {measured_len:.3f} vs tape "
                                   f"{self._expect[0]:.3f}")
            self.report.check("interior width error",
                              abs(measured_wid - self._expect[1]), "<=", tol,
                              "m", f"map {measured_wid:.3f} vs tape "
                                   f"{self._expect[1]:.3f}")

        self.report.check("coverage", coverage, ">=",
                          float(self.get_parameter("min_coverage").value))
        self.report.check("phantom clutter", clutter, "<=",
                          float(self.get_parameter("max_clutter").value), "",
                          "isolated occupied cells over total occupied cells")
        self.report.note(
            "SAVE THIS MAP NOW: ros2 run nav2_map_server map_saver_cli "
            "-f serre_reference. Stages 5 and 6 localise against it.")


def main(args=None) -> None:
    run(Stage4)


if __name__ == "__main__":
    main()
