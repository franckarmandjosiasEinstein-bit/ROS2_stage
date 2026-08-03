"""Stage 3 -- the lidar, standing still, looking at real glass.

The question this stage exists to answer is narrow and important:

    DOES THE LIDAR SEE THE GREENHOUSE?

In simulation, walls are opaque boxes and every beam returns. A real 2D lidar
in a real glasshouse faces three problems that Gazebo does not model:

  * GLASS IS NEARLY INVISIBLE. A beam hitting a pane at close to normal
    incidence gets a weak return or passes through; at a glancing angle it
    reflects away and never comes back. The robot then believes there is no
    wall where there is one, and drives into it. This is the single most
    dangerous sim-to-real difference in this project.
  * HUMIDITY AND MIST scatter the beam: dropouts, and short phantom returns.
  * PLANT LEAVES overhanging the aisle produce returns that are real but
    should not be treated as walls.

The robot does not move. The operator places it at surveyed distances from
known surfaces and the node compares what the lidar reports with what was
measured by tape. Everything here is per-bearing statistics, because a lidar
that works at 0 deg and fails at 90 deg is a lidar that works in the corridor
and fails at the pane, and an aggregate would hide that.
"""

from __future__ import annotations

import math

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

from youbot_commissioning.lib.stage import CommissioningStage, run


class Stage3(CommissioningStage):
    STAGE = 3
    SLUG = "lidar"
    TITLE = "Lidar validation against real surfaces (glass!)"
    PROCEDURE = """
    BEFORE ARMING
      1. The robot DOES NOT MOVE in this stage. No E-stop drill needed, but
         keep it to hand anyway.
      2. Place the robot with its lidar at a measured distance from a GLASS
         pane, square on (beam normal to the glass). Measure with a tape from
         the lidar axis to the glass. 1.0 m is a good first station.

    WHAT WILL HAPPEN
      The node collects 200 scans (about 20 s) and reports, per bearing sector:
        return rate (fraction of beams that came back at all)
        mean range and standard deviation
        the error against the distance you declared

      Declare the expected distance and the bearing you are testing:
        ros2 topic pub -1 /commissioning/expect \\
          std_msgs/Float32MultiArray "{data: [bearing_deg, distance_m]}"

    STATIONS TO RUN (repeat the declaration at each)
      A. 1.0 m from GLASS, square on          <- the critical one
      B. 1.0 m from GLASS, at 45 deg incidence <- usually much worse
      C. 1.0 m from an OPAQUE surface          <- the control
      D. facing down an empty aisle            <- long-range behaviour
      E. facing a plant row                    <- leaf clutter

    PASS MEANS
      Glass is detected at both incidences with an acceptable return rate and
      range error. IF STATION B FAILS, THE GREENHOUSE MUST BE MODIFIED, not
      the software: opaque tape, a kickboard, or a painted band at lidar
      height along every glass surface the robot can approach. Write that in
      the field notes and tell the site.
    """

    def __init__(self):
        super().__init__("stage3_lidar")
        self.declare_parameter("scans", 200)
        self.declare_parameter("sector_deg", 15.0)      # averaging window
        self.declare_parameter("max_range_error", 0.05)  # m
        self.declare_parameter("min_return_rate", 0.90)  # fraction
        self.declare_parameter("max_range_noise", 0.03)  # m, std within sector
        self.declare_parameter("scan_topic", "scan")

        self._scans = 0
        self._want = int(self.get_parameter("scans").value)
        self._expect = None       # (bearing_deg, distance_m)
        self._station = 0
        self._acc: dict[int, list[float]] = {}
        self._sent: dict[int, int] = {}
        self._stations: list[dict] = []
        self._angle_min = None
        self._angle_inc = None
        self._range_max = None

        self.create_subscription(
            LaserScan, str(self.get_parameter("scan_topic").value),
            self._on_scan, 10)
        self.create_subscription(Float32MultiArray, "/commissioning/expect",
                                 self._on_expect, 10)

    def _on_expect(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 2:
            return
        self._expect = (float(msg.data[0]), float(msg.data[1]))
        self._acc.clear()
        self._sent.clear()
        self._scans = 0
        self.get_logger().info(
            f"station {self._station + 1}: expecting {self._expect[1]:.3f} m "
            f"at bearing {self._expect[0]:+.0f} deg -- collecting "
            f"{self._want} scans, hold still")

    def _on_scan(self, msg: LaserScan) -> None:
        if self._angle_min is None:
            self._angle_min = msg.angle_min
            self._angle_inc = msg.angle_increment
            self._range_max = msg.range_max
            self.report.record("beams", len(msg.ranges))
            self.report.record("angle_min_deg", math.degrees(msg.angle_min))
            self.report.record("angle_increment_deg",
                               math.degrees(msg.angle_increment))
            self.report.record("range_max_m", msg.range_max)
        if self._expect is None or self._scans >= self._want:
            return

        sector = float(self.get_parameter("sector_deg").value)
        for i, r in enumerate(msg.ranges):
            bearing = math.degrees(msg.angle_min + i * msg.angle_increment)
            key = int(round(bearing / sector))
            self._sent[key] = self._sent.get(key, 0) + 1
            if math.isfinite(r) and msg.range_min <= r < msg.range_max:
                self._acc.setdefault(key, []).append(float(r))

        self._scans += 1
        if self._scans >= self._want:
            self._finish_station()

    def _finish_station(self) -> None:
        bearing_deg, expected = self._expect
        sector = float(self.get_parameter("sector_deg").value)
        key = int(round(bearing_deg / sector))
        hits = self._acc.get(key, [])
        sent = self._sent.get(key, 0)

        rate = (len(hits) / sent) if sent else 0.0
        mean = (sum(hits) / len(hits)) if hits else None
        std = None
        if hits and mean is not None:
            std = math.sqrt(sum((h - mean) ** 2 for h in hits) / len(hits))
        err = (mean - expected) if mean is not None else None

        station = {
            "bearing_deg": bearing_deg,
            "expected_m": expected,
            "return_rate": rate,
            "mean_range_m": mean,
            "range_std_m": std,
            "range_error_m": err,
            "beams_in_sector": sent,
        }
        self._stations.append(station)
        self._station += 1
        self.get_logger().info(
            f"  station {self._station}: return rate {rate * 100:.1f}%, "
            f"mean {mean if mean is None else round(mean, 3)} m, "
            f"error {err if err is None else round(err, 3)} m, "
            f"noise {std if std is None else round(std, 4)} m")
        self.get_logger().info(
            "declare the next station, or press Ctrl-C to conclude.")
        self._expect = None
        self.report.record("stations", self._stations)

    # Ctrl-C path: lib.stage.run() writes the report. Add the criteria first
    # by overriding stop(), which run() calls before finishing.
    def stop(self) -> None:
        super().stop()
        if self._stations and not self.report.checks:
            self._conclude()

    def _conclude(self) -> None:
        rates = [s["return_rate"] for s in self._stations]
        errs = [abs(s["range_error_m"]) for s in self._stations
                if s["range_error_m"] is not None]
        noises = [s["range_std_m"] for s in self._stations
                  if s["range_std_m"] is not None]

        self.report.check("stations measured", len(self._stations), ">=", 3)
        self.report.check("worst return rate", min(rates) if rates else None,
                          ">=", float(self.get_parameter("min_return_rate").value),
                          "", "a low rate at ONE bearing means glass at that "
                              "incidence is invisible -- mark the pane")
        self.report.check("worst range error", max(errs) if errs else None,
                          "<=", float(self.get_parameter("max_range_error").value),
                          "m")
        self.report.check("worst range noise", max(noises) if noises else None,
                          "<=", float(self.get_parameter("max_range_noise").value),
                          "m")
        self.report.note(
            "If any glass station failed: the fix is physical, not software. "
            "Apply an opaque band at lidar height (0.20 m) along every glass "
            "surface the robot can reach, and re-run this stage.")


def main(args=None) -> None:
    run(Stage3)


if __name__ == "__main__":
    main()
