"""Stage 9 -- one fruit, one arm cycle, under direct supervision.

The last stage, and the one with the smallest scope on purpose. Not "harvest a
row". One berry, chosen by a human, with a hand on the E-stop, repeated enough
times to produce a success rate rather than an anecdote.

WHY SO SMALL
Everything before this stage failed safe: a navigation error stops the robot.
An arm error does not. The arm reaches into the plant, and the failure modes
are damage to the crop, damage to the arm, and damage to whoever is standing
close enough to watch. So the first arm motions near a real plant happen one
at a time, with a person deciding when each one starts.

WHAT IS MEASURED, PER ATTEMPT
    outcome     picked | missed | dropped | aborted
    cycle time  from the go signal to the arm being back at rest
    damage      declared by the operator, and it is a hard fail

The success rate this produces is the first honest number about harvesting in
this project. The simulation figure -- 16 berries, 22 s each -- says nothing
about a real stem, a real leaf, or a berry that is softer than the gripper.

DECLARING AN ATTEMPT
    ros2 topic pub -1 /commissioning/attempt std_msgs/String "{data: go}"
    ...then, after watching it:
    ros2 topic pub -1 /commissioning/attempt std_msgs/String "{data: picked}"
       (or missed / dropped / aborted / damage:<what was damaged>)
"""

from __future__ import annotations

from std_msgs.msg import Bool, String

from youbot_commissioning.lib.stage import CommissioningStage, run


class Stage9(CommissioningStage):
    STAGE = 9
    SLUG = "pick"
    TITLE = "Supervised single-fruit picking"
    PROCEDURE = """
    BEFORE ARMING
      1. Stages 0-8 have PASSED. Stage 8 in particular: an arm driven by a
         detector with two-thirds spurious estimates will reach into leaves.
      2. The robot is STATIONARY and will stay stationary. This stage does not
         drive the base at all.
      3. One operator holds the E-stop and stands where the arm cannot reach.
         A second person calls each attempt.
      4. Agree in advance what "damage" means with whoever owns the crop, and
         agree that the stage stops on the first occurrence.

    WHAT WILL HAPPEN
      Nothing until you send 'go'. Each 'go' triggers exactly ONE arm cycle
      via /pick_request. You watch it, then declare the outcome. The node
      times the cycle and keeps the tally.

    PASS MEANS
      Enough attempts, a success rate above the target, and ZERO damage
      events. Damage is not traded against success rate.
    """

    OUTCOMES = ("picked", "missed", "dropped", "aborted")

    def __init__(self):
        super().__init__("stage9_pick")
        self.declare_parameter("target_attempts", 20)
        self.declare_parameter("min_success_rate", 0.60)
        self.declare_parameter("max_cycle_time", 30.0)     # s
        self.declare_parameter("pick_request_topic", "pick_request")
        self.declare_parameter("pick_done_topic", "pick_done")

        # The base never moves in this stage.
        self._vmax = 0.0
        self._wmax = 0.0

        self._attempts: list[dict] = []
        self._open = None          # the attempt in progress
        self._damage: list[str] = []

        self.pick_pub = self.create_publisher(
            Bool, str(self.get_parameter("pick_request_topic").value), 5)
        self.create_subscription(
            Bool, str(self.get_parameter("pick_done_topic").value),
            self._on_pick_done, 5)
        self.create_subscription(String, "/commissioning/attempt",
                                 self._on_attempt, 10)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_attempt(self, msg: String) -> None:
        text = msg.data.strip().lower()

        if text.startswith("damage"):
            what = text.split(":", 1)[1].strip() if ":" in text else "unspecified"
            self._damage.append(what)
            self.get_logger().error(
                f"DAMAGE DECLARED: {what}. Stopping the stage now.")
            self.report.note(f"DAMAGE: {what}")
            self._conclude()
            self.finish()
            return

        if text == "go":
            if not self.armed:
                self.get_logger().warn("not armed -- arm the stage first")
                return
            if self._open is not None:
                self.get_logger().warn(
                    "an attempt is already open; declare its outcome first")
                return
            self._open = {"index": len(self._attempts) + 1,
                          "t_start": self._now()}
            self.pick_pub.publish(Bool(data=True))
            self.get_logger().info(
                f"attempt {self._open['index']}: pick requested -- WATCH IT, "
                "then declare picked / missed / dropped / aborted")
            return

        if text in self.OUTCOMES:
            if self._open is None:
                self.get_logger().warn("no attempt is open")
                return
            rec = self._open
            rec["outcome"] = text
            rec["cycle_time"] = self._now() - rec["t_start"]
            self._attempts.append(rec)
            self._open = None
            picked = sum(1 for a in self._attempts if a["outcome"] == "picked")
            self.get_logger().info(
                f"  attempt {rec['index']}: {text} in "
                f"{rec['cycle_time']:.1f} s  "
                f"({picked}/{len(self._attempts)} picked so far)")
            if len(self._attempts) >= int(
                    self.get_parameter("target_attempts").value):
                self._conclude()
                self.finish()
            return

        self.get_logger().warn(f"unrecognised declaration '{text}'")

    def _on_pick_done(self, msg: Bool) -> None:
        if self._open is not None:
            self.get_logger().info(
                "  the arm reports the cycle is finished -- declare what you "
                "actually saw (the arm's opinion is not the measurement)")

    def on_disarmed(self) -> None:
        if self._open is not None:
            self._open["outcome"] = "aborted"
            self._open["cycle_time"] = self._now() - self._open["t_start"]
            self._attempts.append(self._open)
            self._open = None

    def stop(self) -> None:
        super().stop()
        if self._attempts and not self.report.checks:
            self._conclude()

    def _conclude(self) -> None:
        n = len(self._attempts)
        picked = sum(1 for a in self._attempts if a["outcome"] == "picked")
        rate = picked / n if n else None
        times = [a["cycle_time"] for a in self._attempts
                 if a.get("cycle_time") is not None]
        worst = max(times) if times else None
        mean = (sum(times) / len(times)) if times else None

        tally = {o: sum(1 for a in self._attempts if a["outcome"] == o)
                 for o in self.OUTCOMES}
        self.report.record("attempts", self._attempts)
        self.report.record("tally", tally)
        self.report.record("success_rate", rate)
        self.report.record("mean_cycle_time_s", mean)
        self.report.record("damage_events", self._damage)

        self.report.check("attempts made", n, ">=",
                          int(self.get_parameter("target_attempts").value))
        self.report.check("success rate", rate, ">=",
                          float(self.get_parameter("min_success_rate").value))
        self.report.check("worst cycle time", worst, "<=",
                          float(self.get_parameter("max_cycle_time").value),
                          "s")
        self.report.check("damage events", len(self._damage), "==", 0,
                          note="not tradeable against success rate")
        self.report.note(
            "This success rate is the first honest harvesting number in the "
            "project. Quote it instead of the simulation figure.")


def main(args=None) -> None:
    run(Stage9)


if __name__ == "__main__":
    main()
