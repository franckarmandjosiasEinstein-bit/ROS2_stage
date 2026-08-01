#!/usr/bin/env python3
"""Pre-flight checks: every failure this project has actually suffered.

Run before a simulation, not after. A 30 minute Gazebo run that dies in the
first second because of a shadowed import, or spends its whole survey fighting
a fence, costs half an hour and a log nobody wants to read. Everything here is
pure Python -- no ROS, no Gazebo -- and the whole suite takes about a second:

    python3 scripts/check_regressions.py

Each check names the run it comes from, so nothing here is hypothetical. When
one fails it prints what it expected, what it found, and where to look.
"""

from __future__ import annotations

import ast
import math
import os
import re
import sys
import xml.dom.minidom

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "src")

FAILURES: list[str] = []
CHECKS = 0


def check(name: str, ok: bool, detail: str = "") -> None:
    global CHECKS
    CHECKS += 1
    if ok:
        print(f"  ok    {name}")
    else:
        print(f"  FAIL  {name}")
        if detail:
            for line in detail.strip().splitlines():
                print(f"          {line}")
        FAILURES.append(name)


def read(*parts) -> str:
    with open(os.path.join(SRC, *parts)) as f:
        return f.read()


# --------------------------------------------------------------------------
def check_xml_wellformed() -> None:
    """`--` inside an XML comment is illegal, and the resulting ExpatError
    names a line number in a generated string, not in the file you edited.
    Cost: two separate debugging sessions."""
    print("\nworld and robot description")
    for rel in ("youbot_gazebo/worlds/greenhouse.sdf",
                "youbot_gazebo/urdf/youbot_gz.urdf"):
        path = os.path.join(SRC, rel)
        if not os.path.exists(path):
            continue
        try:
            xml.dom.minidom.parse(path)
            check(f"{rel} parses", True)
        except Exception as exc:                       # noqa: BLE001
            check(f"{rel} parses", False,
                  f"{exc}\nan XML comment may not contain '--'")


def check_no_shadowed_messages() -> None:
    """slam_node imported nav_msgs' OccupancyGrid and then our own numpy grid
    under the same name. rclpy got a plain Python class and the node died at
    startup with 'This might be a ROS 1 message type', which is a red herring.
    The whole run was lost: no pose, no mission, robot parked at spawn."""
    print("\nROS message classes reaching rclpy")
    for dirpath, _dirs, files in os.walk(SRC):
        for fn in files:
            if not fn.endswith(".py"):
                continue
            path = os.path.join(dirpath, fn)
            with open(path) as f:
                try:
                    tree = ast.parse(f.read())
                except SyntaxError:
                    continue
            bound: dict[str, list[str]] = {}
            for node in ast.walk(tree):
                if isinstance(node, ast.ImportFrom):
                    for a in node.names:
                        bound.setdefault(a.asname or a.name, []).append(
                            f"{node.module} (line {node.lineno})")
            for call in ast.walk(tree):
                if not (isinstance(call, ast.Call)
                        and getattr(call.func, "attr", "") in
                        ("create_publisher", "create_subscription")
                        and call.args
                        and isinstance(call.args[0], ast.Name)):
                    continue
                cls = call.args[0].id
                srcs = bound.get(cls, [])
                rel = os.path.relpath(path, SRC)
                check(f"{rel}: {cls} is not shadowed", len(srcs) <= 1,
                      f"bound {len(srcs)} times: " + "; ".join(srcs)
                      + "\nalias one of them (see mapping_node)")


def check_single_fence() -> None:
    """Two nodes enforcing containment with different numbers is worse than
    one: navigation_node pushed back from 0.25 m before the boundary while
    safety_node only reacted past it, and the robot ping-ponged along
    x = -4.30 for an entire run."""
    print("\ncontainment")
    nav = read("youbot_control/youbot_control/navigation_node.py")
    check("only safety_node owns the fence",
          "fence_x" not in nav and "fence_y" not in nav,
          "navigation_node declares fence parameters again")
    safety = read("youbot_control/youbot_control/safety_node.py")
    check("safety_node has the preventive fence", "_keep_inside" in safety,
          "the fence is corrective only; overshoot cannot be prevented")


def check_brake_is_not_an_override() -> None:
    """Publishing the lidar brake on /safety_override made mission_node throw
    away a berry at offset +0.01 -- dead centre -- because standing next to a
    gutter naturally trips the brake."""
    print("\nsafety_override semantics")
    src = read("youbot_control/youbot_control/safety_node.py")
    # The fence branch publishes True; the normal path must publish False.
    tail = src[src.index("# 1. Swept-corridor protective stop"):]
    check("the brake path does not raise safety_override",
          "Bool(data=True)" not in tail,
          "the normal path publishes an override; only the fence may")


def check_survey_geometry() -> None:
    """The survey waypoints must be reachable at ANY heading: the footprint
    sweeps a 0.347 m corner radius, and a waypoint 0.10 m inside the fence
    trips it the moment the robot turns. The first run of the survey stalled
    on waypoint 0 until it timed out, at truth pose (-4.65, -0.66)."""
    print("\nsurvey circuit")
    mission = read("youbot_control/youbot_control/mission_node.py")
    safety = read("youbot_control/youbot_control/safety_node.py")

    def param(text, name, default):
        m = re.search(rf'declare_parameter\("{name}", ([0-9.]+)\)', text)
        return float(m.group(1)) if m else default

    fx = param(safety, "fence_x", 4.85)
    fy = param(safety, "fence_y", 2.35)
    reach = math.hypot(param(safety, "base_length", 0.58) / 2.0,
                       param(safety, "base_width", 0.38) / 2.0)
    ns: dict = {}
    exec(compile(ast.Module(
        body=[n for n in ast.parse(mission).body
              if isinstance(n, ast.Assign)
              and getattr(n.targets[0], "id", "") in ("SURVEY", "PATROL", "DEPOT")],
        type_ignores=[]), "<mission>", "exec"), ns)

    MARGIN = 0.20          # room for tracking error; the log showed 0.25 m
    for label in ("SURVEY", "PATROL"):
        pts = ns.get(label, [])
        worst = min(((fx - reach) - abs(x), (fy - reach) - abs(y), x, y)
                    for x, y in pts) if pts else None
        ok = all(abs(x) <= fx - reach - MARGIN and abs(y) <= fy - reach - MARGIN
                 for x, y in pts)
        check(f"{label}: every waypoint holds {MARGIN:.02f} m clear of the fence",
              ok,
              f"tightest waypoint ({worst[2]:+.02f}, {worst[3]:+.02f}) leaves "
              f"{worst[0]:+.02f} m in x, {worst[1]:+.02f} m in y; "
              f"need > {MARGIN:.02f}" if worst else "no waypoints found")

    depot = ns.get("DEPOT")
    if depot:
        check("DEPOT is inside the fence at any heading",
              abs(depot[0]) <= fx - reach and abs(depot[1]) <= fy - reach,
              f"depot {depot} against limits "
              f"{fx - reach:.02f} / {fy - reach:.02f}")

    # And the aisles the circuit claims to cover must be the real ones.
    world = read("youbot_gazebo/worlds/greenhouse.sdf")
    gutters = sorted(float(re.search(r"<pose>([^<]+)</pose>", body).group(1).split()[1])
                     for name, body in re.findall(
                         r'<model name="([^"]+)">(.*?)</model>', world, re.S)
                     if name.startswith("gutter_"))
    covered = sorted({round(y, 2) for _x, y in ns.get("SURVEY", [])})
    check("the survey visits one lane per gutter gap",
          len(covered) == len(gutters) + 1,
          f"{len(gutters)} gutters at {gutters} leave {len(gutters) + 1} "
          f"lanes, but the circuit only uses {covered}")


def check_launch_handlers() -> None:
    """A prebuilt ExecuteProcess cannot run twice: calling Shutdown() from a
    process-exit handler re-ran the cleanup action and launch reported
    'executed more than once'. And the exit handler fired on a CLEAN exit,
    printing 'slam_node died' over a normal Ctrl-C."""
    print("\nlaunch files")
    for rel in ("youbot_gazebo/launch/gazebo.launch.py",
                "youbot_slam/launch/gazebo_slam.launch.py",
                "youbot_slam/launch/gazebo_slam_toolbox.launch.py"):
        path = os.path.join(SRC, rel)
        if not os.path.exists(path):
            continue
        text = open(path).read()
        try:
            tree = ast.parse(text)
        except SyntaxError as exc:
            check(f"{rel} parses", False, str(exc))
            continue
        check(f"{rel} parses", True)
        for lam in (n for n in ast.walk(tree) if isinstance(n, ast.Lambda)):
            args = [a.arg for a in lam.args.args]
            check(f"{rel}: event handler takes (event, context)",
                  args == ["event", "context"], f"takes {args}")
        if "OnProcessExit" in text:
            check(f"{rel}: exit handler checks the return code",
                  "returncode" in text,
                  "it will announce a crash on every clean shutdown")
        if "OnShutdown" in text:
            check(f"{rel}: shutdown handler is re-entrant",
                  "on_shutdown=lambda" in text,
                  "a prebuilt action cannot be executed twice")
        # The camera follow must use the supported tracking topic: the
        # deprecated /gui/follow service is why the watchdog could detect the
        # loss of the view and never repair it.
        if "FOLLOW_ROBOT" in text:
            check(f"{rel}: camera uses /gui/track", "/gui/track" in text,
                  "only the deprecated /gui/follow service is used")


def check_vision_thresholds() -> None:
    """The world's near-white specular highlight adds equally to R, G and B:
    it destroys channel RATIOS but never DIFFERENCES. A ratio-based ripeness
    test reported zero fruit for entire runs."""
    print("\nripeness test")
    sys.path.insert(0, os.path.join(SRC, "youbot_control"))
    try:
        import numpy as np
        from youbot_control.lib.vision import red_mask
    except Exception as exc:                            # noqa: BLE001
        check("vision library imports", False, str(exc))
        return
    check("vision library imports", True)

    def lit(rgb, spec):
        return np.clip(np.array(rgb, dtype=np.int16) + spec, 0, 255)

    for spec in (0, 40, 90):
        berry = lit((200, 40, 45), spec).reshape(1, 1, 3).astype(np.uint8)
        check(f"a ripe berry is still ripe under +{spec} specular",
              bool(red_mask(berry)[0, 0]),
              f"pixel {berry[0, 0].tolist()} rejected")
    for name, rgb in (("green leaf", (60, 130, 70)),
                      ("grey gutter", (200, 202, 205)),
                      ("brown soil", (70, 48, 30)),
                      ("dark aisle floor", (38, 64, 54))):
        px = np.array(rgb, dtype=np.uint8).reshape(1, 1, 3)
        check(f"{name} is not fruit", not bool(red_mask(px)[0, 0]),
              f"pixel {rgb} passes the ripeness test")


def check_slam_scoring() -> None:
    """Two scoring bugs, each of which sent the estimate walking backwards:
    a SUM over beams pays for retreating into mapped ground, and a cell grazed
    once by a passing beam was counted as settled evidence so the frontier
    ahead scored as MISSES. 1.20 m of lag in 36 min, then 2.03 m in 13."""
    print("\nscan matcher")
    src = read("youbot_slam/youbot_slam/matcher.py")
    check("the score is a mean, not a sum",
          "total / max(n_known, self.min_beams)" in src,
          "a sum rewards a pose for putting more beams on mapped ground")
    slam = read("youbot_slam/youbot_slam/slam_node.py")
    check("only mature cells vote", "mature_log_odds" in slam,
          "a single grazing beam would count as settled evidence again")
    check("the along-track gain is damped", "along_gain_scale" in slam,
          "the unobservable aisle axis takes corrections at full gain")
    check("divergence from dead reckoning is reported", "max_drift_warn" in slam,
          "a slow walk away from odometry would go unannounced again")
    check("along-track drift is bounded, not just damped",
          "along * self._corr_along > 0.0" in slam,
          "damping alone left a 0.97 m walk, which is more than the mission's "
          "0.30 m arrival tolerance: every leg then runs to its timeout")

    # And the ratchet itself: it must stop a one-way walk while still letting
    # the estimate come back.
    ns: dict = {}
    lim = 0.60
    for name, corr, along, expect in (
            ("under the bound, a push outward is kept", 0.20, -0.05, -0.05),
            ("at the bound, a push further out is refused", -0.60, -0.05, 0.0),
            ("at the bound, a correction back is still applied", -0.60, +0.05, +0.05),
            ("past the bound, coming back is still applied", -0.90, +0.05, +0.05)):
        got = 0.0 if (abs(corr) >= lim and along * corr > 0.0) else along
        check(f"ratchet: {name}", abs(got - expect) < 1e-9,
              f"corr {corr:+.02f}, proposed {along:+.02f} -> kept {got:+.02f}, "
              f"expected {expect:+.02f}")


def check_entry_points() -> None:
    """Every node named in a launch file must actually be installed, or the
    run dies one node at a time with a message about missing metadata."""
    print("\nentry points")
    declared = set()
    for pkg in os.listdir(SRC):
        setup = os.path.join(SRC, pkg, "setup.py")
        if not os.path.exists(setup):
            continue
        for m in re.finditer(r'"(\w+) = ([\w.]+):main"', open(setup).read()):
            declared.add((pkg, m.group(1)))
    for dirpath, _d, files in os.walk(SRC):
        for fn in files:
            if not fn.endswith(".launch.py"):
                continue
            text = open(os.path.join(dirpath, fn)).read()
            for pkg, exe in re.findall(
                    r'package="(youbot_\w+)",\s*executable="(\w+)"', text):
                check(f"{fn}: {pkg}/{exe} is installed",
                      (pkg, exe) in declared,
                      f"no console_script named {exe} in {pkg}/setup.py")
            for exe in re.findall(r'control\("(\w+)"', text):
                check(f"{fn}: youbot_control/{exe} is installed",
                      ("youbot_control", exe) in declared,
                      f"no console_script named {exe}")


def check_preventive_fence() -> None:
    """The behaviour of _keep_inside itself, on the geometry that trapped the
    robot: at (-4.65, -0.66) the footprint is already through the west glass."""
    print("\npreventive fence behaviour")
    sys.path.insert(0, os.path.join(SRC, "youbot_control"))
    src = read("youbot_control/youbot_control/safety_node.py")
    tree = ast.parse(src)
    cls = next(n for n in tree.body
               if isinstance(n, ast.ClassDef) and n.name == "SafetyNode")
    wanted = ("_overhang_at", "_keep_inside")
    ns = {"math": math}
    # Lift the two geometry methods out of the node and exercise them on their
    # own: they are pure functions of a pose and a command, so they can be
    # tested without rclpy, a simulator, or a robot.
    geom = ast.ClassDef(name="Geom", bases=[], keywords=[], decorator_list=[],
                        body=[m for m in cls.body
                              if isinstance(m, ast.FunctionDef)
                              and m.name in wanted])
    module = ast.fix_missing_locations(ast.Module(body=[geom], type_ignores=[]))
    exec(compile(module, "<safety>", "exec"), ns)

    g = ns["Geom"]()
    g._fx, g._fy, g._hl, g._hw, g._look = 4.85, 2.35, 0.29, 0.19, 0.5

    # Deep in the arena nothing is touched.
    g._pose = (0.0, 0.0, 0.0)
    check("open floor: the command passes through unchanged",
          g._keep_inside(0.6, 0.0, 0.4) == (0.6, 0.0, 0.4))

    # Driving west at the west end: the wall-ward component must be dropped.
    g._pose = (-4.40, -0.60, math.pi)          # heading -x
    vx, vy, wz = g._keep_inside(0.6, 0.0, 0.0)
    check("heading into the west wall: forward motion is refused",
          abs(vx) < 1e-6, f"kept vx = {vx:+.03f}")

    # Driving east from the same place is fine.
    g._pose = (-4.40, -0.60, 0.0)              # heading +x
    vx, vy, wz = g._keep_inside(0.6, 0.0, 0.0)
    check("heading away from the wall: allowed", vx > 0.59,
          f"kept only vx = {vx:+.03f}")

    # Turning in place at the tightest legal spot must stay allowed, or the
    # robot can never come about at the end of an aisle.
    g._pose = (-4.25, -1.85, 0.0)
    _, _, wz = g._keep_inside(0.0, 0.0, 0.6)
    check("turning at the end lane is still allowed", abs(wz) > 1e-6,
          "the robot could never turn around")

    # Turning while a corner is already out must not sweep it further.
    g._pose = (-4.70, -0.66, 0.0)
    _, _, wz = g._keep_inside(0.0, 0.0, 1.0)
    ox0, oy0 = g._overhang_at(*g._pose)
    check("already outside: rotation that worsens it is refused",
          abs(wz) < 1e-6 or math.hypot(ox0, oy0) == 0.0,
          f"overhang {ox0:+.03f},{oy0:+.03f} and wz {wz:+.03f} kept")

    # Sideways escape from the wall survives even when forward does not.
    g._pose = (-4.60, 0.0, math.pi)            # heading -x, body +y = world +y
    vx, vy, _ = g._keep_inside(0.5, 0.0, 0.0)
    check("pinned at the wall, the robot is not frozen",
          abs(vx) < 1e-6, f"forward into the wall kept vx = {vx:+.03f}")
    vx, vy, _ = g._keep_inside(-0.5, 0.0, 0.0)
    check("reversing off the wall is allowed", vx < -0.49,
          f"kept vx = {vx:+.03f}")


def check_fruit_projection() -> None:
    """The pixel-to-map projection, on the real camera geometry.

    A gate that rejects everything is as useless as no gate. This exercises
    the actual methods at the poses the harvester works from, and checks both
    halves: a berry beside the robot must be located, and a ray leaving the
    fruit plane at grazing incidence must be refused rather than published as
    a confident position tens of metres away."""
    print("\nfruit localisation")
    src = read("youbot_control/youbot_control/strawberry_detector.py")
    tree = ast.parse(src)
    cls = next(n for n in tree.body
               if isinstance(n, ast.ClassDef) and n.name == "StrawberryDetector")
    wanted = ("_project", "_ray_hit")
    ns = {"math": math}
    geom = ast.ClassDef(name="Proj", bases=[], keywords=[], decorator_list=[],
                        body=[m for m in cls.body
                              if isinstance(m, ast.FunctionDef) and m.name in wanted])
    exec(compile(ast.fix_missing_locations(
        ast.Module(body=[geom], type_ignores=[])), "<detector>", "exec"), ns)

    # The parameter block, read out of the node so the test cannot drift from it.
    params = {m.group(1): float(m.group(2)) for m in re.finditer(
        r'declare_parameter\("(cam_\w+|berry_z|max_range|max_pixel_shift)",'
        r'\s*([0-9.]+)\)', src)}
    params.setdefault("cam_yaw", math.pi / 2.0)

    class P:
        def __init__(self, v):
            self.value = v

    p = ns["Proj"]()
    p.get_parameter = lambda n: P(params[n])
    W, H = 640, 480
    fx = (W / 2.0) / math.tan(params["cam_fov"] / 2.0)

    check("the detector's parameters were all found",
          {"cam_x", "cam_y", "cam_z", "cam_pitch", "cam_fov", "berry_z",
           "max_range", "max_pixel_shift"} <= set(params),
          f"found {sorted(params)}")

    # Robot in the aisle at y = -0.6 facing +x: its camera looks at the row on
    # its left, the gutter at y = 0. A berry dead centre in the frame must be
    # located, and it must land on that row.
    p._pose = (0.0, -0.60, 0.0)
    hit = p._project(W / 2.0, H / 2.0, W, H)
    check("a berry at the centre of the frame is located", hit is not None,
          "the conditioning gate rejects even the best-conditioned pixel")
    if hit:
        check("it lands on the plant row it is looking at", abs(hit[1]) < 0.55,
              f"projected to ({hit[0]:+.02f}, {hit[1]:+.02f}); the row is at y=0")

    # The camera is pitched UP, so pixels BELOW the optical axis are the ones
    # that flatten out: they approach the fruit plane at grazing incidence and
    # eventually point under it entirely. (Pixels above the axis look more
    # steeply upward and are better conditioned, not worse -- which is the
    # opposite of the intuition, and the reason this is a test and not a
    # comment.) Somewhere going down the frame the answer must stop being
    # published.
    worst = None
    for down in range(0, 240, 5):
        if p._project(W / 2.0, H / 2.0 + down, W, H) is None:
            worst = down
            break
    check("grazing rays below the axis are refused, not published",
          worst is not None,
          "every pixel down to the bottom of the frame produced a 'position'")
    if worst is not None:
        check("the gate still accepts a useful part of the frame", worst >= 20,
              f"refuses everything more than {worst} px below the centre")
        print(f"          (usable down to {worst} px below the axis, "
              f"{100.0 * (H / 2.0 + worst) / H:.0f}% of the frame height)")

    # The gate's own claim: what survives must be worth having. Sweep the
    # frame and confirm no accepted pixel is more sensitive than the limit.
    bad = []
    for du in range(-300, 301, 25):
        for dv in range(-200, 201, 25):
            u, v = W / 2.0 + du, H / 2.0 + dv
            a = p._project(u, v, W, H)
            if a is None:
                continue
            b = p._ray_hit(u, v + 1.0, W, H)
            if b is None or math.hypot(b[0] - a[0], b[1] - a[1]) > \
                    params["max_pixel_shift"] + 1e-9:
                bad.append((du, dv))
    check("nothing accepted exceeds the sensitivity limit", not bad,
          f"{len(bad)} accepted pixel(s) move more than "
          f"{params['max_pixel_shift']} m per pixel, e.g. {bad[:3]}")

    # One pixel is worth this much on the ground, at the centre: a sanity
    # figure for the report, and a tripwire if the optics change.
    a = p._ray_hit(W / 2.0, H / 2.0, W, H)
    b = p._ray_hit(W / 2.0, H / 2.0 + 1.0, W, H)
    if a and b:
        print(f"          (one pixel at frame centre = "
              f"{math.hypot(b[0] - a[0], b[1] - a[1]) * 100:.01f} cm on the "
              f"fruit plane, fx = {fx:.0f} px/rad)")


def main() -> int:
    print("pre-flight regression checks")
    check_xml_wellformed()
    check_no_shadowed_messages()
    check_single_fence()
    check_brake_is_not_an_override()
    check_survey_geometry()
    check_launch_handlers()
    check_entry_points()
    check_vision_thresholds()
    check_slam_scoring()
    check_preventive_fence()
    check_fruit_projection()

    print()
    if FAILURES:
        print(f"{len(FAILURES)} of {CHECKS} checks FAILED:")
        for f in FAILURES:
            print(f"  - {f}")
        return 1
    print(f"all {CHECKS} checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
