#!/usr/bin/env python3
"""prettylog -- turn the launch's firehose into something a human reads.

    ros2 launch agri_robot agri.launch.py 2>&1 | python3 tools/prettylog.py

WHY

Bringing this system up prints about 140 lines before the robot has done
anything, and roughly eight of them are about the robot. The rest is Gazebo
announcing every service it advertises, Qt complaining about binding loops in
a file dialog nobody opened, SDF noting that `gz_frame_id` is not in its
schema (it is not; it is a Gazebo extension and it works), and OGRE talking
about visibility masks. All of it is normal, none of it is actionable, and
buried in the middle of it are the three lines that decide whether the run is
going to work at all:

    the robot node could not reach the broker            -> no requests, ever
    agri_viz has not heard /odom                         -> RViz stays empty
    a process died                                       -> something is gone

Those got lost. `ModuleNotFoundError: No module named 'qrcode'` scrolled past
twice in one afternoon and both times the conclusion drawn was "the requests
are not working", which is true and is not the fault.

WHAT IT DOES NOT DO

Throw anything away. The raw log is written to disk by run_sim.sh before this
filter ever sees it, so `--raw` here and `less` on that file both exist. A
filter that is the only copy of its input is a filter you cannot trust.

The rules below are deliberately a flat table rather than anything cleverer.
Each is one observed line from one real run, and when the next unhelpful line
shows up the fix is to add a row, not to understand a framework.
"""

from __future__ import annotations

import argparse
import re
import sys
import time

# ---------------------------------------------------------------- colours
# Disabled when stdout is not a terminal, so a redirected log stays greppable.
class C:
    dim = grey = ok = warn = bad = head = off = ""

    @classmethod
    def enable(cls) -> None:
        cls.grey, cls.dim = "\033[90m", "\033[2m"
        cls.ok, cls.warn, cls.bad = "\033[32m", "\033[33m", "\033[31m"
        cls.head, cls.off = "\033[36m", "\033[0m"


#: Lines matching any of these are dropped. Every one was read, understood,
#: and found to say nothing about whether this run is healthy.
NOISE = [
    r"Binding loop detected",                 # Qt, about a dialog nobody opened
    r"\[QT\]",
    r"XML Element\[gz_frame_id\]",            # a Gazebo extension SDF has no schema for
    r"Trying to serialize component",         # a component with no operator<<
    r"SetVisibilityMask",                     # OGRE reserved bits
    r"Stereo is NOT SUPPORTED",
    r"OpenGl version",
    r"Detected Wayland",
    r"Default logging verbosity",
    r"root link base_link has an inertia",    # KDL, and the workaround is worse
    r"\[Msg\] (Serving|Create service|Remove service|Pose service|Light "
    r"configuration|Physics service|SphericalCoordinates|Enable collision|"
    r"Disable collision|Material service|Publishing|Resource path|Server "
    r"control|Found no publishers|Loaded level|Loading plugin|Move to|Follow|"
    r"Tracking|Camera|Currently tracking|Received world|Loading SDF)",
    r"\[Msg\] (Gazebo Sim (Server|GUI)|JointPositionController|VelocityControl|"
    r"OdometryPublisher)",
    r"\[GUI\] \[Msg\] (Added|Loaded|Using|Listening|Camera|Move|Follow|"
    r"Tracking|Currently)",
    r"process has finished cleanly",
    r"All log files can be found below",
]

#: (pattern, level, template). The template may use \1 \2 for groups; when it
#: is None the line is passed through with its node prefix stripped.
#: Ordered: the first match wins, so put the specific before the general.
RULES: list[tuple[str, str, str | None]] = [
    # --- things that are actually wrong, with the fix attached -----------
    (r"No module named 'qrcode'|No module named 'agri'|No module named 'paho'",
     "bad", "a node started WITHOUT the virtualenv, so it cannot import its "
            "own code. run_sim.sh activates it; if you launched by hand: "
            "source ~/.venvs/agri/bin/activate"),
    (r"cannot reach the broker at (\S+)", "bad",
     r"the robot cannot reach the broker at \1 -- it will keep trying, and "
     r"nothing can order a measurement until it succeeds"),
    (r"still no broker at (\S+)", "bad",
     r"STILL no broker at \1. Requests issued now go nowhere. "
     r"run_sim.sh should have started one -- check that window"),
    (r"process has died \[pid \d+, exit code (-?\d+), cmd '\S*?/?(\w+) ",
     "bad", r"\2 DIED (exit \1) -- look above for its last line"),
    (r"no /odom after", "bad",
     "agri_viz has had no odometry, so RViz has no fixed frame and will stay "
     "empty. The robot is unaffected. Check the gz bridge"),
    (r"lost the broker", "warn",
     "the broker went away; the robot is retrying"),

    # --- our own nodes: already written for a human, keep verbatim -------
    (r"agri_viz: first /odom at (x=\S+ y=\S+);", "ok",
     r"RViz has its fixed frame -- first /odom at \1"),
    (r"agri_viz: publishing", "info",
     "agri_viz up: publishing map -> base_link and the catalogue markers"),
    (r"(agri_robot \S+: keys in .*)$", "ok", r"\1"),
    # The route optimiser and the odometer both log through the ROS logger
    # like everything else, which means without a row here they vanish
    # exactly like everything else this file exists to hide -- and these
    # two are the numbers a demonstration of the optimisation is FOR. Found
    # by watching a real run: the mission finished in a third of the time
    # and the terminal gave no reason why.
    (r"robot: (\S+) reordered to save (.*)$", "ok",
     r"mission \1 reordered: saved \2"),
    (r"robot: (\S+) \S+ \(\d+/\d+\), ([\d.]+ m driven)$", "ok",
     r"mission \1 \2"),
    (r"(RECONNECTED to .*)$", "warn", r"\1"),
    (r"(connected to \S+, listening on .*)$", "ok", r"\1"),
    (r"reaching for the broker at (\S+)", "info", r"waiting for the broker at \1"),
    (r"\[view\] (.*)$", "info", r"\1"),

    # --- milestones ------------------------------------------------------
    (r"Robot initialized", "ok", "robot description loaded"),
    (r"Entity creation successful", "ok", "youbot spawned in the greenhouse"),
    (r"World \[(\w+)\] initialized", "ok", r"world '\1' running"),
    (r"Creating (?:GZ->ROS|ROS->GZ) Bridge: \[(/\S+)", "bridge", None),
    (r"rviz2\]: Stereo", "drop", None),

    # --- the mission, from robot_node ------------------------------------
    (r"(\w{12}): (\d+) station\(s\)$", "ok",
     r"mission \1 accepted: \2 station(s)"),
    (r"(\w{12}): done in (.*)$", "ok", r"mission \1 finished in \2"),
    (r"mission aborted: (.*)$", "bad", r"mission ABORTED: \1"),
]

#: Everything that survives NOISE and matches no rule is shown only if it
#: carries one of these. A launch says a great many true and uninteresting
#: things, and the default has to be quiet or this filter buys nothing.
KEEP_ANYWAY = re.compile(r"\[(ERROR|FATAL|WARN)\]|Traceback|Error:|error:|"
                         r"refused|Refused|FAILED|failed to")

PREFIX = re.compile(r"^\[([a-zA-Z_0-9]+)-\d+\] ")
STAMP = re.compile(r"\[\d{9,}\.\d+\] ")
LEVEL = re.compile(r"^\[(INFO|WARN|ERROR|FATAL|DEBUG)\] ")

MARK = {"ok": "ok  ", "info": "·   ", "warn": "warn", "bad": "FAIL",
        "head": "▶   "}
COLOUR = {"ok": lambda: C.ok, "info": lambda: C.grey, "warn": lambda: C.warn,
          "bad": lambda: C.bad, "head": lambda: C.head}


class Filter:
    def __init__(self, raw: bool) -> None:
        self.raw = raw
        self.noise = re.compile("|".join(NOISE))
        self.rules = [(re.compile(p), lvl, tpl) for p, lvl, tpl in RULES]
        self.bridged: list[str] = []
        self.started: list[str] = []
        self.dropped = 0

    def say(self, level: str, text: str) -> None:
        col = COLOUR[level]()
        print(f"{C.grey}{time.strftime('%H:%M:%S')}{C.off} "
              f"{col}{MARK[level]}{C.off}  {text}", flush=True)

    def emit(self, level: str, text: str) -> None:
        """Say one line, after any collected group that must precede it.

        The two groups -- which processes launch started, which topics the
        bridge opened -- are collected rather than printed one per line, and
        a group is only complete once a line that is not part of it arrives.
        Emitting through here is what keeps them in the right PLACE: held
        until then, but printed before whatever ended them, so the log still
        reads top to bottom in the order things happened.
        """
        if self.bridged:
            names = ", ".join(t.lstrip("/") for t in self.bridged)
            self.say("ok", f"gz bridge: {len(self.bridged)} topics ({names})")
            self.bridged = []
        self.say(level, text)

    def feed(self, line: str) -> None:
        line = line.rstrip("\n")
        if self.raw:
            print(line, flush=True)
            return
        if not line.strip():
            return

        # "[gazebo-1] [Msg] ..." -> node name and the rest.
        m = PREFIX.match(line)
        body = line[m.end():] if m else line
        body = STAMP.sub("", body)
        body = re.sub(r"^\[(INFO|WARN|ERROR|FATAL)\] \[[\d.]+\] ", "", body)
        body = re.sub(r"^\[[\w.]+\]: ", "", body)

        if "process started with pid" in line:
            who = re.search(r"\[(\w+)-\d+\]: process started", line)
            if who:
                self.started.append(who.group(1))
            return
        if self.started:
            self.say("head", "started: " + ", ".join(self.started))
            self.started = []

        if self.noise.search(line):
            self.dropped += 1
            return

        for pat, level, tpl in self.rules:
            hit = pat.search(body) or pat.search(line)
            if not hit:
                continue
            if level == "drop":
                self.dropped += 1
                return
            if level == "bridge":
                self.bridged.append(hit.group(1))
                return
            self.emit(level, hit.expand(tpl) if tpl else body)
            return

        if KEEP_ANYWAY.search(line):
            self.emit("warn" if "WARN" in line else "bad", body)
            return
        self.dropped += 1

    def flush_summaries(self) -> None:
        """Anything still collected when the stream ends."""
        if self.started:
            self.say("head", "started: " + ", ".join(self.started))
            self.started = []
        if self.bridged:
            names = ", ".join(t.lstrip("/") for t in self.bridged)
            self.say("ok", f"gz bridge: {len(self.bridged)} topics ({names})")
            self.bridged = []


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--raw", action="store_true",
                    help="pass everything through unchanged")
    ap.add_argument("--no-colour", action="store_true")
    args = ap.parse_args(argv)

    if sys.stdout.isatty() and not args.no_colour:
        C.enable()

    f = Filter(args.raw)
    try:
        for line in sys.stdin:
            f.feed(line)
    except KeyboardInterrupt:
        # Ctrl-C reaches this filter too, and it must not die first: the
        # cleanup lines run_sim.sh prints AFTER the launch stops are the ones
        # saying what was killed and where the log went. Keep draining.
        try:
            for line in sys.stdin:
                f.feed(line)
        except KeyboardInterrupt:
            pass
    finally:
        f.flush_summaries()
        if f.dropped and not args.raw:
            print(f"{C.grey}{f.dropped} routine lines hidden; the full log is "
                  f"the file named above{C.off}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
