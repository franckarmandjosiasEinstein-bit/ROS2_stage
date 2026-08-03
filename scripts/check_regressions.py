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


def check_no_crabbing() -> None:
    """The path follower must not translate sideways along its path.

    Reported from the Gazebo window before it was found in the code: "the
    robot moves diagonally instead of going straight". pure_pursuit resolved
    the world-frame velocity into the body frame and let the base crab toward
    the lookahead point while also turning toward it. A 0.58 x 0.38 m body
    crossing an aisle at 45 deg sweeps 0.68 m of it, and -- worse -- the
    protective stop tests the corridor along the COMMANDED direction, so an
    oblique command aims the test rectangle into the gutter beside the robot
    instead of down the lane ahead. 46% of that run was spent blocked inside
    1.05 m lanes."""
    print("\npath following: heading before translation")
    sys.path.insert(0, os.path.join(SRC, "youbot_control"))
    from youbot_control.lib.pure_pursuit import PurePursuit

    pp = PurePursuit(cruise_speed=0.6, lookahead=0.32)
    pp.set_path([(0.0, 0.0), (4.0, 0.0)])          # due east

    # Pointed along the path: full speed, straight ahead, no yaw command.
    st, vx, vy, wz = pp.step(0.0, 0.0, 0.0)
    check("aligned with the path: drives forward", st == "running" and vx > 0.5,
          f"vx = {vx:+.03f}")
    check("aligned with the path: no sideways component", abs(vy) < 1e-9,
          f"vy = {vy:+.03f} -- this is the crab")
    check("aligned with the path: no needless yaw", abs(wz) < 1e-6,
          f"wz = {wz:+.03f}")

    # Facing 90 deg off: the base must turn, not slide sideways.
    pp.set_path([(0.0, 0.0), (4.0, 0.0)])
    _, vx, vy, wz = pp.step(0.0, 0.0, math.pi / 2)
    check("90 deg off: does not translate", abs(vx) < 1e-6 and abs(vy) < 1e-6,
          f"vx = {vx:+.03f}, vy = {vy:+.03f}")
    check("90 deg off: turns toward the path", wz < -0.5,
          f"wz = {wz:+.03f} (expected a negative, clockwise, command)")

    # Half-aligned: speed scaled by cos, still no sideways component.
    pp.set_path([(0.0, 0.0), (4.0, 0.0)])
    _, vx, vy, _ = pp.step(0.0, 0.0, math.pi / 3)   # 60 deg off
    check("partly aligned: speed is scaled, not redirected",
          0.0 < vx < 0.35 and abs(vy) < 1e-9,
          f"vx = {vx:+.03f}, vy = {vy:+.03f}")

    # Target behind: never translate away from it.
    pp.set_path([(0.0, 0.0), (4.0, 0.0)])
    _, vx, vy, _ = pp.step(0.0, 0.0, math.pi)
    check("facing away: refuses to translate", abs(vx) < 1e-9 and abs(vy) < 1e-9,
          f"vx = {vx:+.03f}, vy = {vy:+.03f}")

    # And the guard must be able to say WHAT it braked for.
    safety = read("youbot_control/youbot_control/safety_node.py")
    check("the protective stop names the obstacle it stopped for",
          "blocking_point" in safety and "_why_blocked" in safety,
          "a brake that logs only its threshold cannot be debugged from a log")


def check_params_match_code() -> None:
    """The YAML must not silently disagree with the node's own defaults.

    The run of 2026-08-03 14:13. lib/clearance.py changed safety_node's
    distances from sensor-relative to bumper-relative and moved the defaults
    from 0.28 / 0.60 / 0.30 down to 0.12 / 0.35 / 0.10 -- but youbot_params.yaml
    still carried the old numbers, and the YAML wins. 0.28 m in front of the
    BUMPER is 0.57 m from the centre: every threshold doubled. The robot braked
    for anything within half a metre, spent 58% of the run blocked (against 6%
    on the first lap of the previous build), and picked 5 strawberries where it
    had picked 27. Nothing crashed and nothing looked wrong in the launch.

    So: every parameter that appears in BOTH places must carry the same value.
    Tuning is still fine -- change it in both, which is the point."""
    print("\nparameters: YAML vs code")
    yaml_path = os.path.join(SRC, "youbot_bringup/config/youbot_params.yaml")
    text = open(yaml_path).read()

    # Tiny reader for the one shape this file has: "node:" / "ros__parameters:"
    # / "  key: value  # comment". Not a YAML parser, and does not need to be.
    sections, node = {}, None
    for line in text.splitlines():
        if re.match(r"^[a-z_]+:\s*$", line):
            node = line.rstrip(":").strip()
            sections[node] = {}
            continue
        m = re.match(r"^\s{4}([a-z_]+):\s*(-?[0-9.]+)\s*(#.*)?$", line)
        if m and node:
            sections[node][m.group(1)] = float(m.group(2))

    check("the parameter file was parsed", any(sections.values()),
          f"no key/value pairs found in {yaml_path}")

    pkg = {"mapping_node": "youbot_control", "planning_node": "youbot_control",
           "navigation_node": "youbot_control", "safety_node": "youbot_control"}
    total = mismatched = 0
    for node, params in sections.items():
        if node not in pkg:
            continue
        src = read(f"{pkg[node]}/{pkg[node]}/{node}.py")
        for name, want in params.items():
            m = re.search(r'declare_parameter\(\s*"%s"\s*,\s*(-?[0-9.]+)'
                          % re.escape(name), src)
            if not m:
                continue        # declared elsewhere or not a plain number
            total += 1
            if abs(float(m.group(1)) - want) > 1e-9:
                mismatched += 1
                check(f"{node}.{name} agrees with the code", False,
                      f"YAML says {want}, {node}.py declares {m.group(1)} -- "
                      "the YAML wins at runtime, so the code default is a lie")
    check("every tuned parameter matches its node default", mismatched == 0,
          f"{mismatched} of {total} disagree")
    if mismatched == 0:
        print(f"          ({total} parameters cross-checked)")


def check_station_realign() -> None:
    """The station sweep must actually get a second berry.

    MAX_PICKS_PER_STOP is 3, yet all 27 picks of the 2026-08-03 run logged
    "Station: 1 picked here". The re-align path set four of the six alignment
    clocks and left `_align_gain_t` and `_align_best_abs` at the previous
    berry's values, so the stall test fired on the first tick: "next berry at
    offset +0.42 -> re-aligning" then, half a second of sim later, "Alignment
    stalled at offset +0.42 for 4s". Every time."""
    print("\nstation multi-pick")
    src = read("youbot_control/youbot_control/mission_node.py")
    tree = ast.parse(src)
    cls = next(n for n in tree.body
               if isinstance(n, ast.ClassDef) and n.name == "MissionNode")
    fns = {m.name: m for m in cls.body if isinstance(m, ast.FunctionDef)}

    check("the alignment clocks are armed in one place", "_arm_align" in fns,
          "a helper must own them, or a caller will forget one again")
    if "_arm_align" not in fns:
        return
    armed = {n.attr for n in ast.walk(fns["_arm_align"])
             if isinstance(n, ast.Attribute) and isinstance(n.ctx, ast.Store)}
    needed = {"_align_start", "_align_sign", "_align_check_t",
              "_align_last_abs", "_align_best_abs", "_align_gain_t"}
    check("it arms every clock the stall test reads", needed <= armed,
          "missing: " + ", ".join(sorted(needed - armed)))

    # Both entries into an alignment must go through it: the first berry of a
    # station, and the re-align onto the next one.
    for fn in ("_begin_align", "_update_pick"):
        body = ast.dump(fns[fn])
        check(f"{fn} arms the clocks through the helper",
              "'_arm_align'" in body,
              f"{fn} sets alignment state by hand -- that is the bug")
        stale = [n.attr for n in ast.walk(fns[fn])
                 if isinstance(n, ast.Attribute) and isinstance(n.ctx, ast.Store)
                 and n.attr in needed]
        check(f"{fn} does not set them by hand as well", not stale,
              "still assigns " + ", ".join(sorted(set(stale))))


def check_bumper_clearance() -> None:
    """Distances must be measured from the FOOTPRINT, not from the lidar.

    The run of 2026-08-03: the robot buried its nose in the east wall and the
    stuck detector fired ten times at (+3.70, +0.61) over nine minutes.
    stop_distance was 0.28 m measured from the lidar, which sits at the centre
    of a 0.58 m base -- so the brake fired when the wall was 1 cm INSIDE the
    front bumper. The base has no collision geometry, so nothing else was ever
    going to stop it."""
    print("\nbumper-relative clearance")
    sys.path.insert(0, os.path.join(SRC, "youbot_control"))
    from youbot_control.lib.clearance import (best_escape, corridor_clearance,
                                              footprint_reach)

    HL, HW, BAND = 0.29, 0.19, 0.24

    check("footprint reach forward is half the length",
          abs(footprint_reach(0.0, HL, HW) - 0.29) < 1e-9,
          f"got {footprint_reach(0.0, HL, HW):.3f}")
    check("footprint reach sideways is half the width",
          abs(footprint_reach(math.pi / 2, HL, HW) - 0.19) < 1e-9,
          f"got {footprint_reach(math.pi / 2, HL, HW):.3f}")
    # A rectangle, not the 0.347 m circumscribed circle: braking on that would
    # stop the robot for a wall it is merely driving alongside.
    check("reach at 45 deg is the rectangle, not the circle",
          footprint_reach(math.pi / 4, HL, HW) < 0.28,
          f"got {footprint_reach(math.pi / 4, HL, HW):.3f}")

    # The wedge itself: a wall 0.30 m ahead of the SENSOR is 0.01 m ahead of
    # the bumper. The old code called that "0.30 m of room" and kept driving.
    wall = [(0.30, y * 0.05) for y in range(-10, 11)]
    gap = corridor_clearance(wall, 0.0, BAND, HL, HW)
    check("a wall 0.30 m from the lidar reads as 0.01 m from the bumper",
          abs(gap - 0.01) < 1e-6, f"read {gap:.3f} m")

    # And the guard must actually be using this. safety_node had its own copy
    # measuring raw beam range; one number in two places is how they drift.
    safety = read("youbot_control/youbot_control/safety_node.py")
    check("safety_node brakes on the shared bumper clearance",
          re.search(r"from youbot_control\.lib\.clearance import "
                    r"[^\n]*\bcorridor_clearance\b", safety) is not None
          and "corridor_clearance(self._pts" in safety,
          "safety_node must not re-implement the clearance")
    stop = re.search(r'"stop_distance", ([0-9.]+)', safety)
    check("stop_distance is a bumper clearance, not a sensor range",
          stop is not None and float(stop.group(1)) < HL,
          f"found {stop.group(1) if stop else 'nothing'} -- anything >= {HL} "
          "would mean the number is being read as a range from the lidar")

    # Open aisle: 0.8 m wide, robot centred. It must NOT brake -- an earlier
    # version spent a whole lap crawling down a clear corridor.
    aisle = [(x * 0.05, 0.40) for x in range(-80, 81)] + \
            [(x * 0.05, -0.40) for x in range(-80, 81)]
    check("centred in an 0.8 m aisle, the corridor ahead is clear",
          corridor_clearance(aisle, 0.0, BAND, HL, HW) == float("inf"),
          "the robot would brake in open corridor")

    # The escape. A pocket: wall ahead, wall behind, gutter to the left, the
    # only way out is a sideways strafe. The old fixed manoeuvre was always
    # (-0.15, 0, 0.6) -- straight back, into 0.05 m of room, which the guard
    # then braked to nothing. That is the ten-times loop.
    pocket = [(0.34, y * 0.05) for y in range(-12, 13)] + \
             [(-0.34, y * 0.05) for y in range(-12, 13)] + \
             [(x * 0.05, 0.42) for x in range(-12, 13)]
    back = corridor_clearance(pocket, math.pi, BAND, HL, HW)
    check("in the pocket, reversing really is blocked",
          back < 0.10, f"reverse clearance {back:.3f} m")
    d, gap = best_escape(pocket, BAND, HL, HW, avoid=0.0)
    check("the escape finds the open side instead", d is not None and gap > 1.0,
          f"chose {math.degrees(d) if d is not None else None} deg, "
          f"{gap:.2f} m free")
    check("the escape strafes sideways, not backwards",
          d is not None and abs(abs(math.degrees(d)) - 90.0) < 31.0,
          f"chose {math.degrees(d) if d is not None else None} deg")
    delta = abs(math.atan2(math.sin(d), math.cos(d)))
    check("the escape never repeats the heading that is already failing",
          delta > math.radians(59.0), f"only {math.degrees(delta):.0f} deg away")

    # Fully boxed in: no direction has room. It must say so rather than
    # returning a confident answer -- the caller then rotates in place.
    box = [(0.34 * math.cos(a * 0.05), 0.34 * math.sin(a * 0.05))
           for a in range(0, 126)]
    _, gap = best_escape(box, BAND, HL, HW, avoid=None)
    check("boxed in on every heading, no direction is oversold", gap < 0.20,
          f"claimed {gap:.2f} m of room")

    # And the loop has to terminate somewhere: the follower reports a goal it
    # cannot reach, the mission drops it. Without that the only exit was the
    # 60 s timeout, twice, for nine minutes of log.
    nav = read("youbot_control/youbot_control/navigation_node.py")
    mission = read("youbot_control/youbot_control/mission_node.py")
    check("the follower reports a goal it cannot escape toward",
          'create_publisher(Bool, "goal_blocked"' in nav,
          "navigation_node must publish goal_blocked")
    check("the mission abandons a goal reported blocked",
          'create_subscription(Bool, "goal_blocked"' in mission
          and "_abandon_goal()" in mission,
          "mission_node must subscribe to goal_blocked")
    check("the escape uses the same clearance the guard brakes with",
          "from youbot_control.lib.clearance import best_escape" in nav,
          "or the guard will veto the direction the escape just chose")


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
    check_bumper_clearance()
    check_no_crabbing()
    check_params_match_code()
    check_station_realign()
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
