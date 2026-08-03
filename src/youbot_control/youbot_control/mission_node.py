"""mission_node -- high-level orchestrator (patrol -> discover -> collect).

Sensor-driven mission: the robot patrols coverage waypoints while a perception
source (the real vision_node in Webots, or sim_node's FOV cone headless)
publishes /detected_crates. Newly seen crates are remembered; once the patrol
is done the robot drives to each, "picks" it (a TODO placeholder for the arm
action), and finally delivers at the depot.

Subscribes:  /odom (nav_msgs/Odometry), /detected_crates (geometry_msgs/PoseArray)
Publishes:   /goal_pose (geometry_msgs/PoseStamped)
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Int32, Float32, Float32MultiArray, Bool

from youbot_control.lib.align_lqr import AlignController
from youbot_control.lib.fruit_map import FruitMap

# Coverage patrol: only the two central 0.8 m aisles (Y = +/-0.6). The robot
# turns to face its heading, so its left (+Y, where the camera and arm are)
# always points at a flanking plant row -- the camera never faces a wall, and
# these two aisles still flank all three gutters. (Margins are skipped: there,
# the left side can be the outer wall -> a blank view and picking in the void.)
PATROL = [
    (-3.8, 0.6), (3.8, 0.6),      # +X: left camera sees the Y=+1.2 row
    (3.8, -0.6), (-3.8, -0.6),    # -X: left camera sees the Y=-1.2 row
]

# SURVEY -- the mapping phase, run BEFORE any harvesting.
#
# Two separate jobs were being asked of one route and neither was getting
# done. PATROL is a harvesting route: it only visits the two central aisles,
# because those are the ones where the left-hand camera always faces a plant
# row. As a MAPPING route it is hopeless -- the outer aisles at y = +/-1.85
# were never driven, so a third of the greenhouse never entered the map, and
# the planner then had to route the robot through territory it had never
# seen. Hence "it only ever explores one aisle".
#
# This is the coverage route, and it is a plain boustrophedon over all four
# aisles plus both end lanes. Geometry it is derived from (greenhouse.sdf):
#   walls      x = +/-5.0, y = +/-2.5  (0.1 thick -> inner faces 4.95 / 2.45)
#   gutters    y = -1.2, 0.0, +1.2, each 8.0 long (x in [-4, 4]) and 0.4 wide
# so the four free aisles are centred on y = -1.85, -0.60, +0.60, +1.85 and
# the ends (|x| > 4.0) are open across the full width.
#
# The x = +/-4.40 of the first version left only 0.10 m of slack: the
# footprint sweeps a 0.347 m corner radius, the fence sits at 4.85, so the
# centre limit is 4.503 -- and pure pursuit overshot by 0.25 m on the very
# first leg. The robot ended up at (-4.65, -0.66) with a corner through the
# glass, and spent the whole 60 s goal timeout being shoved back in. Every
# waypoint now keeps 0.20 m clear of that limit on both axes, which
# scripts/check_regressions.py enforces.
SURVEY = [
    (-4.25, -1.78), (4.25, -1.78),    # south margin, west -> east
    (4.25, -0.60), (-4.25, -0.60),    # aisle 2, east -> west
    (-4.25, 0.60), (4.25, 0.60),      # aisle 3, west -> east
    (4.25, 1.78), (-4.25, 1.78),      # north margin, east -> west
]
SURVEY_LAPS = 1            # laps of the circuit before scouting begins.
                           # Was 3. map_eval measured the greenhouse at
                           # -4 cm / +6 cm with 100% coverage and 0.0% clutter
                           # after lap ONE; laps 2 and 3 cost 163 s of sim time
                           # between them and improved nothing measurable. That
                           # time now buys corroborated fruit instead.
SCOUT_LAPS = 2             # laps of PATROL that LOOK but never pick.
                           # Two, because that is the minimum that can supply a
                           # second, independent viewpoint -- see lib/fruit_map.
FRUIT_FUSION = 0.25        # m: detections closer than this are one cluster.
                           # Fruit position error is 0.21 m and berries on one
                           # plant sit 0.05-0.20 m apart, so the map resolves
                           # CLUSTERS, not berries. Pretending otherwise would
                           # split one berry into ghosts at every observation.
FRUIT_MIN_PASSES = 2       # distinct scouting passes before a cluster is real
FRUIT_MIN_BASELINE = 0.40  # m between the viewpoints of those passes

DEPOT = (4.3, 1.9)         # east end of the wide top corridor. (4.6, 0.0) sat
                           # in the pinch between the inflated gutter ends and
                           # the inflated east wall -> the robot jammed there.
ARRIVAL_TOLERANCE = 0.30   # m
DEDUP_DIST = 0.6           # m: merge detections into one crate
GOAL_TIMEOUT = 60.0        # s: abandon a goal we can't reach, advance to next
                           # (long enough for the far depot diagonal)
PICK_TIMEOUT = 20.0        # s: max wait for one arm pick cycle before resuming
RIPE_MIN = 1               # ripe clusters in view to stop and align
LOST_GRACE = 2.0           # s the fruit may be absent before alignment is
                           # abandoned (a berry near the frame edge flickers)
COMMIT_OFFSET = 0.85       # |offset| beyond this = at the very edge of frame;
                           # stopping for it wastes a station, because it
                           # leaves the frame as soon as the robot creeps
CENTER_TOL = 0.15          # fruit this near the image centre = aligned -> pick
PICK_SPACING = 0.7         # m: min travel between two picks (plants are ~0.9 m
                           # apart, so this lets it pick almost every plant)
ALIGN_KP = 0.28            # m/s per unit offset (proportional centring)
ALIGN_VMIN = 0.025         # m/s: floor so it keeps creeping near the target
ALIGN_VMAX = 0.10          # m/s: cap
ALIGN_TIMEOUT = 12.0       # s: give up aligning if it can't centre the fruit
ALIGN_STALL = 4.0          # s without the offset improving -> give up. The
                           # timeout alone is far too patient: the log shows
                           # 12 s of sim time (38 s on the wall at this
                           # real-time factor) spent creeping at a berry whose
                           # offset sat at +0.04..+0.08 and never centred,
                           # with the robot visibly motionless the whole time.
ALIGN_PROGRESS = 0.03      # |offset| must improve by this much to count
# Global watchdog. Every per-state timeout above can only fire while that
# state is being ticked; if the machine ever lands somewhere none of them
# cover, the mission goes silent (observed in the field: silent for 80 s
# after 'Goal reached'). This is the backstop that always makes progress.
WATCHDOG = 75.0            # s without a state change -> force the next goal
HEARTBEAT = 15.0           # s between state reports

# Multi-pick per station: like commercial harvesters, pick everything within
# reach before moving on (driving costs time). After each pick the mission
# looks for the NEXT cluster in the frame, sweeping in one direction only so
# already-picked berries (now behind) are never re-targeted.
MAX_PICKS_PER_STOP = 3     # arm cycles per stop before resuming the patrol
NEXT_MIN = 0.18            # |offset| below this = the berry just picked
NEXT_MAX = 1.20            # |offset| beyond this = out of the arm's reach
TRACK_WIN = 0.40           # per-frame tracking window while re-aligning


class MissionNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_node")
        self._pose = None
        self._known = []        # confirmed crate positions (x, y)
        self._collected = []    # crates already visited
        self._patrol_i = 0
        self._survey_i = 0       # waypoint index within the survey circuit
        self._survey_lap = 0     # completed survey laps
        self._scout_lap = 0      # completed scouting laps
        # The memory the robot did not have. Until now nothing accumulated the
        # detector's per-frame world positions, so a red pixel seen once was
        # acted upon as if it were fruit: truth_monitor measured 66 of 111 map
        # estimates matching no real berry, and the harvest spent 39% of its
        # time creeping toward them and timing out. Those are the same number.
        self._fruit = FruitMap(fusion_radius=FRUIT_FUSION,
                               min_passes=FRUIT_MIN_PASSES,
                               min_baseline=FRUIT_MIN_BASELINE)
        # survey -> explore -> collect -> deliver -> done. Mapping first: the
        # harvest is only as good as the map the planner is driving on.
        self._phase = "survey"
        self._goal = None        # (x, y) current goal
        self._goal_kind = None   # "explore" | "pick" | "depot"
        self._goal_sent = False
        self._goal_time = None   # wall-clock secs when the goal was sent
        self._ripe = 0           # ripe fruit clusters currently in view
        self._ripe_seen_t = 0.0  # last time fruit was actually in the frame
        self._overridden = False  # safety_node is vetoing our commands
        self._ripe_offset = 2.0  # horizontal offset of nearest fruit (0 = beside)
        self._picking = False    # arm is running a pick cycle
        self._pick_done = False
        self._pick_start = None
        self._last_pick_xy = None  # where the last pick happened (spacing)
        self._aligning = False   # creeping to centre a spotted fruit
        self._align_start = None
        self._align_sign = 1.0
        self._align_last_abs = None
        self._align_check_t = 0.0
        self._align_best_abs = None   # best |offset| this alignment
        self._align_gain_t = 0.0      # when it last improved
        self._offsets = []       # ALL cluster offsets in the current frame
        self._picks_at_stop = 0  # picks done at the current station
        # The alignment loop. Period matches the mission tick; v_max keeps the
        # old ceiling, so what changes is the LAW, not the speed budget.
        self._align_ctrl = AlignController(
            period=0.1, settling_time=1.5, damping=0.9, v_max=ALIGN_VMAX,
            image_gain_ref=1.0, range_ref=1.0)
        self._ripe_range = None      # metres to the targeted berry, if known
        self._sweep_sign = 0.0   # station sweep direction (0 = not set yet)
        self._station_target = None  # offset of the berry being re-aligned to
        self._laps = 0               # completed harvest rounds
        self._state_sig = None       # last observed state signature
        self._state_since = 0.0      # when the state last changed

        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(PoseArray, "detected_crates", self._on_detections, 10)
        self.create_subscription(Int32, "ripe_count", self._on_ripe, 10)
        self.create_subscription(Bool, "safety_override", self._on_override, 5)
        self.create_subscription(Float32, "ripe_offset", self._on_ripe_offset, 10)
        self.create_subscription(Float32MultiArray, "ripe_offsets", self._on_ripe_offsets, 10)
        self.create_subscription(Empty, "pick_done", self._on_pick_done, 5)
        self.create_subscription(Bool, "goal_blocked", self._on_blocked, 5)
        self.goal_pub = self.create_publisher(PoseStamped, "goal_pose", 10)
        # World positions of the fruit seen this frame, from the detector.
        # They are only ever FOLDED INTO THE MAP -- never acted on directly.
        self.create_subscription(PoseArray, "berry_detections",
                                 self._on_berry_detections, 5)
        self.lap_pub = self.create_publisher(Int32, "survey_lap", 5)
        self.pick_pub = self.create_publisher(Empty, "do_pick", 5)
        self.hold_pub = self.create_publisher(Bool, "pick_hold", 5)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_timer(0.5, self._tick)
        self.create_timer(HEARTBEAT, self._heartbeat)
        self.get_logger().info(
            f"mission_node up: {SURVEY_LAPS} mapping lap, then {SCOUT_LAPS} "
            "SCOUTING passes that look but never pick, then harvesting only "
            "the clusters corroborated from two viewpoints.")

    # --- perception -------------------------------------------------
    def _on_odom(self, msg: Odometry) -> None:
        self._pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_detections(self, msg: PoseArray) -> None:
        for p in msg.poses:
            c = (p.position.x, p.position.y)
            if all(math.hypot(c[0] - k[0], c[1] - k[1]) > DEDUP_DIST for k in self._known):
                self._known.append(c)
                self.get_logger().info(
                    f"Vision: new crate at ({c[0]:+.2f}, {c[1]:+.2f}) "
                    f"[{len(self._known)} known].")

    def _on_blocked(self, msg: Bool) -> None:
        """navigation_node has tried and failed to escape the same spot
        several times: this goal is not reachable from where we are. Advance
        immediately. Waiting out GOAL_TIMEOUT instead is what turned one bad
        corner into nine minutes of the log, twice."""
        if not msg.data or self._goal is None or self._picking or self._aligning:
            return
        self.get_logger().warn(
            f"[{self._phase}] goal {self._goal_kind} "
            f"({self._goal[0]:+.2f}, {self._goal[1]:+.2f}) reported blocked by "
            "the follower -- abandoning, advancing.")
        self._abandon_goal()

    def _on_override(self, msg: Bool) -> None:
        """The guard is overriding us. Nothing we command is reaching the
        wheels, so alignment cannot converge -- stop asking."""
        self._overridden = bool(msg.data)

    def _pass_id(self) -> int:
        """Which scouting pass we are in. Survey counts as pass 0.

        The pass number is what makes a second sighting INDEPENDENT: two
        observations from the same drive-by are one piece of evidence however
        many frames they span.
        """
        return self._scout_lap + 1 if self._phase == "scout" else 0

    def _on_berry_detections(self, msg: PoseArray) -> None:
        """Fold this frame's world positions into the map. Never act on them.

        Acting on a single frame is exactly what produced 66 spurious map
        estimates out of 111. The map decides what is real; this callback only
        supplies evidence to it.
        """
        if self._pose is None or self._phase not in ("scout", "explore"):
            return
        t = self._now()
        view = (self._pose[0], self._pose[1])
        pid = self._pass_id()
        for pose in msg.poses:
            self._fruit.observe(pose.position.x, pose.position.y,
                                pid, view, t)

    def _on_ripe(self, msg: Int32) -> None:
        self._ripe = int(msg.data)
        if self._ripe >= RIPE_MIN:
            self._ripe_seen_t = self._now()

    def _on_ripe_offset(self, msg) -> None:
        self._ripe_offset = float(msg.data)

    def _on_ripe_offsets(self, msg) -> None:
        self._offsets = list(msg.data)

    def _on_pick_done(self, _msg: Empty) -> None:
        self._pick_done = True

    # --- state watchdog ---------------------------------------------
    def _signature(self):
        return (self._phase, self._patrol_i, self._survey_i, self._goal,
                self._goal_sent, self._aligning, self._picking,
                self._picks_at_stop)

    def _heartbeat(self) -> None:
        """Say where the mission is, so a stall is visible instead of silent."""
        if self._phase == "done":
            return
        pos = ("?" if self._pose is None
               else f"({self._pose[0]:+.2f}, {self._pose[1]:+.2f})")
        goal = "none" if self._goal is None else \
            f"{self._goal_kind} ({self._goal[0]:+.2f}, {self._goal[1]:+.2f})"
        state = ("aligning" if self._aligning else
                 "picking" if self._picking else "driving")
        if self._phase == "survey":
            where = (f"waypoint {self._survey_i}/{len(SURVEY)} "
                     f"lap {self._survey_lap + 1}/{SURVEY_LAPS}")
        elif self._phase == "scout":
            where = (f"waypoint {self._patrol_i}/{len(PATROL)} "
                     f"pass {self._scout_lap + 1}/{SCOUT_LAPS}")
        else:
            where = f"waypoint {self._patrol_i}/{len(PATROL)}"
        # The map's state belongs in the heartbeat: "1 ripe in view" says
        # nothing about whether the robot BELIEVES it, and that belief is now
        # what decides whether it stops.
        fm = (f" | map {len(self._fruit.unpicked())} confirmed/"
              f"{len(self._fruit.clusters)} proposed"
              if self._fruit.clusters else " | map empty")
        self.get_logger().info(
            f"[state] {self._phase}/{state} at {pos} -> goal {goal} "
            f"| {where} | {self._ripe} ripe in view{fm}")

    def _check_watchdog(self) -> None:
        sig, t = self._signature(), self._now()
        if sig != self._state_sig:
            self._state_sig, self._state_since = sig, t
            return
        if t - self._state_since < WATCHDOG:
            return
        self.get_logger().warn(
            f"[watchdog] no progress for {WATCHDOG:.0f}s in {self._phase} "
            "-- releasing the base and advancing to the next goal.")
        self._release_base()
        self._abandon_goal()
        self._state_since = t

    # --- mission loop -----------------------------------------------
    def _tick(self) -> None:
        # A mission with no pose does NOTHING and says nothing: no goal, no
        # command, and the watchdog below never runs either because it lives
        # past this guard. From the outside that is indistinguishable from a
        # crash -- the robot simply sits there. Say what we are waiting for.
        if self._pose is None:
            self.get_logger().warn(
                "No pose yet on 'odom' (remapped to /pose_slam under SLAM) -- "
                "the mission cannot start. Check slam_node is alive and "
                "publishing: ros2 topic hz /pose_slam",
                throttle_duration_sec=5.0)
            return
        if self._phase == "done":
            return
        self._check_watchdog()
        if self._picking:
            self._publish_cmd(0.0, 0.0, 0.0)  # base held still while the arm works
            self._update_pick()
            return
        if self._aligning:
            self._update_align()
            return
        if self._goal is None:
            self._choose_goal()
            return
        if not self._goal_sent:
            self._send_goal()
            return
        # See a strawberry while driving a rang -> STOP and align to it, then pick
        # (spaced out so one cluster isn't picked repeatedly).
        if (self._phase == "explore" and self._ripe >= RIPE_MIN
                and abs(self._ripe_offset) <= COMMIT_OFFSET
                and self._far_from_last_pick()
                and self._fruit_here_is_real()):
            self._begin_align()
            return
        if math.hypot(self._goal[0] - self._pose[0], self._goal[1] - self._pose[1]) < ARRIVAL_TOLERANCE:
            self._on_arrival()
        elif self._goal_time is not None and \
                (self.get_clock().now().nanoseconds * 1e-9 - self._goal_time) > GOAL_TIMEOUT:
            self.get_logger().warn(
                f"[{self._phase}] goal {self._goal_kind} "
                f"({self._goal[0]:+.2f}, {self._goal[1]:+.2f}) timed out "
                f"after {GOAL_TIMEOUT:.0f}s -- abandoning, advancing.")
            self._abandon_goal()

    def _fruit_here_is_real(self) -> bool:
        """Has the map corroborated fruit anywhere the robot could reach?

        THIS IS THE GATE THE WHOLE FRUIT MAP EXISTS FOR. Without it the mission
        stops for whatever the current frame shows, which is how it came to
        spend 39% of the harvest phase creeping toward things that were not
        fruit and then timing out.

        The test is deliberately POSITIONAL rather than per-detection: the
        detector says "there is something red at this bearing", the map says
        "there is corroborated fruit at this place". Matching them by position
        means a spurious detection standing where a real cluster is still gets
        acted on -- which is correct, there IS fruit there -- while the same
        detection over bare gutter does not.

        Fails OPEN when the map is empty. If scouting produced nothing at all,
        something is broken upstream and a silent robot that never picks is a
        worse failure than the one being fixed; it says so instead.
        """
        if not self._fruit.clusters:
            if not getattr(self, "_warned_empty_map", False):
                self._warned_empty_map = True
                self.get_logger().warn(
                    "the fruit map is EMPTY after scouting -- no detections "
                    "reached it. Falling back to reacting per frame, which is "
                    "the old behaviour and its old false-positive rate. Check "
                    "that /berry_detections is publishing and that the camera "
                    "head reports a settled angle.")
            return True
        # The camera looks along the robot's left flank, so a berry in frame is
        # roughly abeam. Search a generous radius around the robot rather than
        # trying to invert the bearing: the map's job is to veto places, not to
        # localise this particular blob.
        for c in self._fruit.unpicked():
            if math.hypot(c.x - self._pose[0], c.y - self._pose[1]) <= 1.6:
                return True
        return False

    def _far_from_last_pick(self) -> bool:
        if self._last_pick_xy is None:
            return True
        return math.hypot(self._pose[0] - self._last_pick_xy[0],
                          self._pose[1] - self._last_pick_xy[1]) > PICK_SPACING

    def _choose_goal(self) -> None:
        if self._phase == "survey":
            if self._survey_i < len(SURVEY):
                self._set_goal(SURVEY[self._survey_i], "survey")
                return
            self._survey_lap += 1
            self._survey_i = 0
            self.lap_pub.publish(Int32(data=self._survey_lap))
            if self._survey_lap < SURVEY_LAPS:
                self.get_logger().info(
                    f"[survey] lap {self._survey_lap}/{SURVEY_LAPS} complete "
                    "-- going round again.")
                self._set_goal(SURVEY[0], "survey")
                return
            self.get_logger().info(
                f"[survey] {SURVEY_LAPS} lap(s) done, the greenhouse is mapped "
                f"-- switching to {SCOUT_LAPS} SCOUTING laps (look, do not "
                "pick).")
            self._phase = "scout"
            self._patrol_i = 0
        if self._phase == "scout":
            if self._patrol_i < len(PATROL):
                self._set_goal(PATROL[self._patrol_i], "scout")
                return
            self._scout_lap += 1
            self._patrol_i = 0
            if self._scout_lap < SCOUT_LAPS:
                self.get_logger().info(
                    f"[scout] pass {self._scout_lap}/{SCOUT_LAPS} done, "
                    f"{len(self._fruit.clusters)} clusters proposed so far "
                    "-- going round again for a second viewpoint.")
                self._set_goal(PATROL[0], "scout")
                return
            for line in self._fruit.report_lines():
                self.get_logger().info("[fruit] " + line)
            self.get_logger().info(
                f"[scout] {SCOUT_LAPS} passes done -- harvesting "
                f"{len(self._fruit.unpicked())} CORROBORATED clusters. "
                "Detections that never got a second viewpoint are ignored "
                "from here on.")
            self._phase = "explore"
        if self._phase == "explore":
            if self._patrol_i < len(PATROL):
                self._set_goal(PATROL[self._patrol_i], "explore")
                return
            self._phase = "collect"
        if self._phase == "collect":
            remaining = [c for c in self._known if c not in self._collected]
            if remaining:
                nearest = min(remaining, key=lambda c: math.hypot(
                    c[0] - self._pose[0], c[1] - self._pose[1]))
                self._set_goal(nearest, "pick")
                return
            self._phase = "deliver"
        if self._phase == "deliver":
            self._set_goal(DEPOT, "depot")

    def _on_arrival(self) -> None:
        kind = self._goal_kind
        if kind == "survey":
            self._survey_i += 1
        elif kind == "explore":
            # Picking happens on-the-fly while driving (see _tick); at the
            # waypoint we simply move on to the next one.
            self._patrol_i += 1
        elif kind == "pick":
            self._collected.append(self._goal)
            self.get_logger().info(
                f"At crate ({self._goal[0]:+.2f}, {self._goal[1]:+.2f}). "
                f"TODO: pick with the arm. [{len(self._collected)}/{len(self._known)}]")
        elif kind == "depot":
            # A harvester does not stop after one lap: unload, then go round
            # again. (Idling at the depot is what filled the last field log
            # with ten minutes of nothing.)
            self._laps += 1
            self.get_logger().info(
                f"Unloaded at the depot -- lap {self._laps} done. Starting the "
                "next harvesting round.")
            self._patrol_i = 0
            self._phase = "explore"
            self._last_pick_xy = None
        self._goal = None
        self._goal_sent = False
        self._goal_time = None

    def _abandon_goal(self) -> None:
        """Give up on an unreachable goal and move the mission forward."""
        kind = self._goal_kind
        if kind == "survey":
            self._survey_i += 1
        elif kind == "explore":
            self._patrol_i += 1
        elif kind == "pick":
            # Mark it "handled" so collect doesn't loop on it forever.
            self._collected.append(self._goal)
        # depot: just fall through and let deliver re-issue the goal.
        self._goal = None
        self._goal_sent = False
        self._goal_time = None

    # --- align then pick --------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_cmd(self, vx, vy, wz) -> None:
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        self.cmd_pub.publish(t)

    def _arm_align(self, offset: float) -> None:
        """(Re)start the alignment clocks for a target at `offset`.

        ALL of them, every time. The station sweep used to set only four of the
        six, leaving `_align_gain_t` at the moment the PREVIOUS berry last
        improved -- ten seconds earlier -- and `_align_best_abs` at that berry's
        centred value of ~0.05. So the stall test fired on the very first tick
        of the new alignment: the field log shows "next berry at offset +0.42
        -> re-aligning" followed 0.5 s of sim later by "Alignment stalled at
        offset +0.42 for 4s", every single time. MAX_PICKS_PER_STOP is 3 and
        the log says "Station: 1 picked here" on all 27 picks -- the
        multi-pick-per-station feature has never once run. That is the single
        biggest throughput loss in the run: the robot drives to a plant with a
        dozen ripe berries in frame, takes one, and leaves."""
        # The integral state belongs to ONE target. Carrying a charge
        # across berries would drive the base off toward where the last
        # one was, which is exactly the class of bug the six clocks
        # below already exist to prevent.
        self._align_ctrl.reset()
        self._align_start = self._now()
        self._align_sign = 1.0
        self._align_check_t = self._now()
        self._align_last_abs = abs(offset)
        self._align_best_abs = abs(offset)
        self._align_gain_t = self._now()

    def _begin_align(self) -> None:
        """Spotted a fruit: stop, take over the base, creep to centre it."""
        self._aligning = True
        self._arm_align(self._ripe_offset)
        self._picks_at_stop = 0                  # new station
        self._sweep_sign = 0.0
        self._station_target = None              # first target = nearest cluster
        self._last_pick_xy = self._pose          # space the next detection from here
        self.hold_pub.publish(Bool(data=True))   # navigation yields the base
        self.get_logger().info(
            f"Fruit spotted (offset {self._ripe_offset:+.2f}) -> stopping to align.")

    def _update_align(self) -> None:
        t = self._now()
        if self._station_target is None:
            # First berry of the station: steer on the nearest-to-centre offset.
            off = self._ripe_offset
        else:
            # Re-aligning to the NEXT berry: the nearest-to-centre offset would
            # point back at the one just picked, so track OUR target from frame
            # to frame instead (closest offset to its last known position).
            cands = [o for o in self._offsets
                     if abs(o - self._station_target) < TRACK_WIN]
            if not cands:
                self._publish_cmd(0.0, 0.0, 0.0)
                self.get_logger().info("Next berry lost from view, resuming patrol.")
                self._release_base()
                return
            self._station_target = min(cands, key=lambda o: abs(o - self._station_target))
            off = self._station_target
        # Lost the fruit or took too long -> give up and resume the patrol.
        #
        # "Lost" has to mean GONE, not "absent from one frame". The old test
        # was `self._ripe < RIPE_MIN`, and a berry near the edge of the frame
        # drops out for a frame or two all the time -- the field log is full of
        # "Fruit spotted (offset +0.92)" followed one second later by
        # "Alignment lost", fifteen times in a run, each costing a full stop
        # and a replan storm. Give the detection LOST_GRACE seconds to come
        # back before abandoning.
        if (t - self._ripe_seen_t) > LOST_GRACE or (t - self._align_start) > ALIGN_TIMEOUT:
            self._publish_cmd(0.0, 0.0, 0.0)
            self.get_logger().info("Alignment lost/timed out, resuming patrol.")
            self._release_base()
            return
        if self._overridden:
            self._publish_cmd(0.0, 0.0, 0.0)
            self.get_logger().info(
                "Outside the arena -- this berry is not reachable from here, "
                "resuming patrol.")
            self._release_base()
            return
        # Not converging. Standing still for the full timeout while the offset
        # refuses to shrink harvests nothing and reads, from the Gazebo window,
        # exactly like a frozen robot.
        if abs(off) < self._align_best_abs - ALIGN_PROGRESS:
            self._align_best_abs = abs(off)
            self._align_gain_t = t
        elif t - self._align_gain_t > ALIGN_STALL:
            self._publish_cmd(0.0, 0.0, 0.0)
            self.get_logger().info(
                f"Alignment stalled at offset {off:+.2f} for {ALIGN_STALL:.0f}s "
                "-- resuming patrol.")
            self._release_base()
            return
        # Centred -> pick.
        if abs(off) < CENTER_TOL:
            self._publish_cmd(0.0, 0.0, 0.0)
            self._aligning = False
            self._begin_pick()
            return
        # Every ~1.2 s, if the offset isn't shrinking, the drive sense is wrong
        # -> flip it (robust to the unknown camera->motion sign).
        if t - self._align_check_t > 1.2:
            if abs(off) > self._align_last_abs - 0.02:
                self._align_sign *= -1.0
                # The integral was charging the wrong way; carrying it across
                # a sign flip would make it fight the corrected direction.
                self._align_ctrl.reset()
            self._align_last_abs = abs(off)
            self._align_check_t = t

        # STATE FEEDBACK, not a proportional creep.
        #
        # The old law was v = clamp(KP*|off|, VMIN, VMAX)*sign(off), and its
        # signature fills every log: "Alignment stalled at offset +0.26 for
        # 4s". That is a STEADY-STATE ERROR, and a proportional law with a
        # floor cannot remove one -- below VMIN/KP the speed is floored rather
        # than zeroed, so the base nudges forever without converging.
        #
        # lib/align_lqr models the loop as X = [offset, integral(offset)] with
        # the lateral speed as input, places the poles for a 1.5 s settling
        # time at zeta = 0.9, and COMPUTES the static gain instead of hoping
        # for it. Replayed against the three stalls in the 23:31 log it reaches
        # exactly zero where the old law leaves 0.16 to 0.29 behind.
        #
        # It also scales the gain by 1/range: a berry at 0.5 m crosses the
        # frame four times faster than one at 2 m for the same sideways
        # motion, which a fixed gain ignores -- jumpy near, sluggish far.
        v, _diag = self._align_ctrl.step(off, distance=self._ripe_range)
        self._publish_cmd(self._align_sign * v, 0.0, 0.0)

    def _begin_pick(self) -> None:
        self._picking = True
        self._pick_done = False
        self._pick_start = self._now()
        self.pick_pub.publish(Empty())        # base stays held (hold_pub already True)
        self.get_logger().info(
            f"Aligned at ({self._pose[0]:+.2f}, {self._pose[1]:+.2f}) -> picking.")

    def _update_pick(self) -> None:
        if not (self._pick_done or (self._now() - self._pick_start) > PICK_TIMEOUT):
            return
        if not self._pick_done:                   # arm timed out -> bail out
            self.get_logger().info(f"Pick timed out ({PICK_TIMEOUT:.0f}s), resuming patrol.")
            self._picking = False
            self._release_base()
            return
        self._picks_at_stop += 1
        self._picking = False
        # Tell the map. Without this the same cluster stays "unpicked" and the
        # gate keeps opening for it long after the fruit is in the basket.
        if self._pose is not None:
            self._fruit.mark_picked(self._pose[0], self._pose[1])
        # Station sweep: anything else within reach? Pick it before moving --
        # driving costs time, so harvest everything reachable from this stop.
        # One sweep direction only, so picked berries (now behind) are never
        # re-targeted.
        if self._picks_at_stop < MAX_PICKS_PER_STOP:
            cands = [o for o in self._offsets
                     if NEXT_MIN < abs(o) < NEXT_MAX
                     and (self._sweep_sign == 0.0 or o * self._sweep_sign > 0.0)]
            if cands:
                target = min(cands, key=abs)
                self._sweep_sign = 1.0 if target > 0.0 else -1.0
                self._station_target = target
                self._aligning = True
                self._arm_align(target)
                self.get_logger().info(
                    f"Station: {self._picks_at_stop} picked here, next berry at "
                    f"offset {target:+.2f} -> re-aligning.")
                return
        self.get_logger().info(
            f"Pick done ({self._picks_at_stop} at this stop), resuming patrol.")
        self._release_base()

    def _release_base(self) -> None:
        """Give the base back to navigation and resume the current patrol goal."""
        self.hold_pub.publish(Bool(data=False))
        self._aligning = False
        self._picking = False
        self._station_target = None
        self._last_pick_xy = self._pose       # space the next stop from HERE
        self._goal_sent = False               # re-send the patrol goal to resume

    def _set_goal(self, xy, kind) -> None:
        self._goal = xy
        self._goal_kind = kind
        self._goal_sent = False

    def _send_goal(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(self._goal[0])
        msg.pose.position.y = float(self._goal[1])
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self._goal_sent = True
        self._goal_time = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(
            f"[{self._phase}] goal {self._goal_kind} -> "
            f"({self._goal[0]:+.2f}, {self._goal[1]:+.2f}).")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass      # RuntimeError: message mid-shutdown (rclpy)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
