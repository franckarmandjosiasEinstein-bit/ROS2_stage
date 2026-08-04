#!/usr/bin/env python3
"""read_run -- the verdict on a run, extracted from its log.

    ros2 launch youbot_slam gazebo_slam.launch.py 2>&1 | tee run.log
    python3 scripts/read_run.py run.log

WHY THIS EXISTS

Six changes are stacked in this stack that have never run: the alignment
state feedback, the new head-settled semantics, the drivetrain model, the
dataset capture, the fruit map, and the exact corridor geometry. Judging
whether they worked by scrolling a thirty-minute log is how the important
line gets missed -- and it was missed once already, when `none located`
appeared on every single detection for nine minutes and the run was allowed
to continue.

So the reading grid is code. Each item below states what it is looking for,
what the answer means, and -- where there is one -- the number from the last
measured run to beat. Nothing here interprets: it reports what the log says
and what that implies, and says plainly when the log does not contain the
answer at all.

A missing answer is NOT a pass. It is reported as UNKNOWN, because the most
expensive failure in this project was a subsystem that was silently off.
"""

from __future__ import annotations

import math
import re
import sys

OK, BAD, WARN, UNK = "PASS", "FAIL", "PARTIAL", "UNKNOWN"

COLOUR = {OK: "\033[32m", BAD: "\033[31m", WARN: "\033[33m", UNK: "\033[35m"}
RESET = "\033[0m"


class Item:
    def __init__(self, name, why):
        self.name, self.why = name, why
        self.verdict, self.detail = UNK, "no line in the log answers this"


def head(lines, out):
    """Is the pan head usable at all?

    The head is upstream of everything visual: while it is not settled, the
    detector refuses to publish a fruit position, so a head that never
    settles turns the whole perception chain off without any error message.
    That is exactly what happened -- 'settled' required the measured angle to
    land within 0.02 rad of the target, and the joint had no damping, so it
    never entered the window.
    """
    it = Item("head settles", "perception is gated on it")
    steady = [l for l in lines if "head steady at" in l]
    moving = [l for l in lines if "head moving, left" in l]
    mismatch = [l for l in lines if "steady at" in l and "but was asked for" in l]
    if not steady and not moving:
        it.detail = ("camera_pan_node said nothing -- check it is running and "
                     "that it is reading gz_joint_states")
    elif steady:
        it.verdict = OK
        it.detail = f"{len(steady)} settle events, {len(moving)} in motion"
        if mismatch:
            it.verdict = WARN
            it.detail += (f"; {len(mismatch)} settled AWAY from the target "
                          "-- the head is stable but not where it was asked "
                          "to be, so bearings are biased")
    else:
        it.verdict = BAD
        it.detail = ("the head moved but never settled -- every detection "
                     "will be refused downstream")
    out.append(it)


def located(lines, out):
    """Do detections turn into world positions?

    'none located' with a reason is the message that ran for nine minutes
    while nothing was wrong with the detector. The two reasons are different
    faults: no pose means localisation is not publishing, ray misses the
    fruit plane means the geometry or the head angle is wrong.
    """
    it = Item("fruit is localised", "a bearing is not a position")
    ok = sum(len(re.findall(r"(\d+) located on the map", l)) for l in lines)
    n_ok = sum(int(m) for l in lines
               for m in re.findall(r"(\d+) located on the map", l))
    no_pose = sum("none located (no pose yet)" in l for l in lines)
    no_ray = sum("none located (ray misses the fruit plane)" in l for l in lines)
    if not (ok or no_pose or no_ray):
        it.detail = "the detector never reported on localisation"
    elif n_ok and not (no_pose + no_ray):
        it.verdict = OK
        it.detail = f"{n_ok} detections placed on the map, no failures"
    elif n_ok:
        it.verdict = WARN
        it.detail = (f"{n_ok} placed, but {no_pose} refused for want of a "
                     f"pose and {no_ray} because the ray missed the fruit "
                     "plane")
    else:
        it.verdict = BAD
        it.detail = (f"NOTHING was localised: {no_pose} 'no pose yet', "
                     f"{no_ray} 'ray misses the fruit plane'. Stop the run "
                     "-- perception is mute and nothing downstream means "
                     "anything.")
    out.append(it)


def alignment(lines, out):
    """Did the state feedback remove the steady-state error?

    Baseline to beat: stalls at offsets of +0.16 to +0.29, which is what a
    proportional law leaves when its correction falls below the speed floor.
    A stall at a LARGE offset is a different fault -- an actuator limit, not
    a gain -- and no controller fixes it.
    """
    it = Item("alignment converges", "39% of harvest time went here")
    stalls = [float(m) for l in lines
              for m in re.findall(r"Alignment stalled at offset ([+-][\d.]+)", l)]
    aligned = sum("Aligned at" in l for l in lines)
    if not stalls and not aligned:
        it.detail = "the mission never entered alignment"
    elif not stalls:
        it.verdict = OK
        it.detail = f"{aligned} alignments, no stalls"
    else:
        small = [s for s in stalls if abs(s) < 0.35]
        big = [s for s in stalls if abs(s) >= 0.35]
        it.verdict = BAD if small else WARN
        it.detail = (f"{len(stalls)} stalls ({aligned} succeeded). "
                     f"{len(small)} at small offsets |e| < 0.35 "
                     f"{'-- the steady-state error is NOT gone' if small else ''}"
                     f"; {len(big)} at large offsets, which is the actuator "
                     "limit and wants COMMIT_OFFSET, not a gain")
    out.append(it)


def fruitmap(lines, out):
    """Did corroboration actually throw anything away?

    Baseline: 66 of 111 map estimates matched no real berry. If the map
    confirms nearly everything it was offered, the two-pass rule is not
    biting and the harvest will chase the same phantoms as before.
    """
    it = Item("fruit map corroborates", "66/111 estimates were phantoms")
    conf = [l for l in lines if "confirmed" in l and "passes" in l]
    obs = [l for l in lines if re.search(r"detections -> \d+ clusters", l)]
    empty = [l for l in lines if "fruit map is EMPTY after scouting" in l]
    if empty:
        it.verdict = BAD
        it.detail = ("the map is empty after scouting -- the gate has failed "
                     "OPEN by design, so the harvest is running unguarded")
    elif conf and obs:
        m = re.search(r"(\d+) detections -> (\d+) clusters", obs[-1])
        c = re.search(r"confirmed (\d+)", conf[-1])
        if m and c:
            clusters, confirmed = int(m.group(2)), int(c.group(1))
            share = 100.0 * confirmed / clusters if clusters else 0.0
            it.verdict = OK if share < 80.0 else WARN
            it.detail = (f"{m.group(1)} detections -> {clusters} clusters, "
                         f"{confirmed} confirmed ({share:.0f}%). "
                         + ("the rule is biting" if share < 80 else
                            "almost everything was confirmed -- the rule is "
                            "not rejecting anything, check the baseline"))
    elif conf:
        it.verdict = WARN
        it.detail = conf[-1].strip()[-90:]
    else:
        it.detail = "no scouting summary -- did the run reach the harvest?"
    out.append(it)


def guard(lines, out):
    """Is the guard still holding translation beside the gutters?

    The corridor geometry was replaced because a constant band braked for
    walls the robot was driving alongside. Held translations should now be
    rare and should happen where something really is in the way.
    """
    it = Item("guard is not over-braking", "the corridor geometry changed")
    held = sum("Translation held" in l for l in lines)
    override = sum("Outside the arena" in l for l in lines)
    if not any("safety_node" in l for l in lines):
        it.detail = "safety_node produced no output"
    else:
        it.verdict = OK if held < 20 else WARN
        it.detail = (f"{held} held translations, {override} fence overrides. "
                     + ("plausible" if held < 20 else
                        "still a lot -- check WHERE, the guard now names the "
                        "blocking point"))
    out.append(it)


def moving(lines, out):
    """Did the robot actually MOVE?

    This item exists because a run passed every other test while the robot
    stood still for twelve minutes. truth_monitor reported the same pose
    (-4.45, 1.86, 93 deg) forty times over, the mission issued goals, the
    planner planned to them, and nothing anywhere said "the base is not
    moving". The reading grid did not catch it either, which is the more
    embarrassing half.

    The truth pose is the honest witness: it comes from the simulator, not
    from any estimator that could itself be the thing that died.
    """
    it = Item("the robot moves", "a frozen base passes every other test")
    poses = re.findall(r"truth pose ([-+]?[\d.]+), ([-+]?[\d.]+)", "".join(lines))
    if len(poses) < 3:
        it.detail = "fewer than three truth reports -- too short to tell"
        out.append(it)
        return
    pts = [(float(x), float(y)) for x, y in poses]
    # Longest run of consecutive IDENTICAL truth poses.
    longest, run = 1, 1
    for a, b in zip(pts, pts[1:]):
        run = run + 1 if a == b else 1
        longest = max(longest, run)
    total = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
    if longest >= 5:
        it.verdict = BAD
        # Name the culprit, not the symptom. Three different faults produce an
        # identical frozen truth pose, and each now leaves its own line in the
        # log, so the grid can say WHICH instead of costing another
        # thirty-minute run to find out. Last time it said "look for POSE
        # STALE", the answer was that POSE STALE was absent, and that told
        # nobody anything.
        stale = any("POSE STALE" in l for l in lines)
        pinned = [l for l in lines if "PINNED" in l]
        held = any("Base held by mission_node" in l for l in lines)
        turn = [l for l in lines if "did not turn" in l]
        if stale:
            why = ("'POSE STALE' is in the log: the pose source (slam_node) "
                   "went silent and the follower stopped on purpose.")
        elif pinned:
            why = ("'PINNED': the preventive fence is holding the base "
                   "against the arena boundary. " + pinned[-1].strip()[-120:])
        elif turn:
            why = ("the follower commanded a rotation and the base did not "
                   "turn: " + turn[-1].strip()[-120:])
        elif held:
            why = ("mission_node never released pick_hold -- it died mid-pick "
                   "and nothing has driven since.")
        else:
            why = ("and NOTHING in the log says why: no 'POSE STALE', no "
                   "'PINNED', no 'Stuck'. That is a NEW failure mode, and it "
                   "needs a message of its own before it is worth debugging "
                   "twice.")
        it.detail = (f"FROZEN: the same truth pose {pts[-1]} repeated "
                     f"{longest} times in a row. {why}")
    elif total < 5.0:
        it.verdict = WARN
        it.detail = (f"only {total:.1f} m of truth motion across the whole "
                     "run -- barely moved")
    else:
        it.verdict = OK
        it.detail = (f"{total:.1f} m of truth motion, longest stationary "
                     f"stretch {longest} reports")
    out.append(it)


def timing(lines, out):
    """The time budget. Baseline: 39% of the harvest phase in failed aligns."""
    it = Item("time budget", "39% working is the number to beat")
    final = [l for l in lines if "--- time & throughput (final)" in l]
    pct = [l for l in lines if re.search(r"driving.*blocked.*working", l)]
    if not final:
        it.detail = ("perf_monitor printed no final report -- it prints on "
                     "shutdown, so let the run end rather than killing it")
    else:
        it.verdict = OK
        it.detail = (pct[-1].strip()[:110] if pct
                     else "final report present, percentages not parsed")
    out.append(it)


def main(argv) -> int:
    if len(argv) != 2:
        print(__doc__)
        return 2
    with open(argv[1], errors="replace") as fh:
        lines = fh.readlines()

    items: list[Item] = []
    for fn in (head, located, moving, alignment, fruitmap, guard, timing):
        fn(lines, items)

    print(f"\nrun verdict -- {argv[1]}, {len(lines)} lines\n")
    width = max(len(i.name) for i in items)
    for i in items:
        c = COLOUR[i.verdict]
        print(f"  {c}{i.verdict:<8}{RESET} {i.name:<{width}}  {i.why}")
        for chunk in [i.detail[j:j + 68] for j in range(0, len(i.detail), 68)]:
            print(f"           {' ' * width}  {chunk}")
        print()

    fails = [i for i in items if i.verdict == BAD]
    unknown = [i for i in items if i.verdict == UNK]
    if any(i.name == "fruit is localised" and i.verdict == BAD for i in items):
        print("  STOP. Perception is mute. Nothing measured after this point "
              "means anything.\n")
        return 1
    if fails:
        print(f"  {len(fails)} of 7 failed. Fix those before reading the "
              "numbers.\n")
        return 1
    if unknown:
        print(f"  {len(unknown)} of 7 UNKNOWN -- the log does not say. That "
              "is not a pass.\n")
        return 1
    print("  Nothing in the grid failed. The numbers are worth comparing.\n")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
