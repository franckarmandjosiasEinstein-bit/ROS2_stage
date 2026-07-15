"""Unit tests for the framework-independent algorithm libraries.

These run WITHOUT ROS or Webots -- just `pytest`. They are the fast safety
net that let us port the Webots code with confidence. Run from the package:
    colcon test --packages-select youbot_control
or directly:
    python3 -m pytest src/youbot_control/test -v
"""

import numpy as np

from youbot_control.lib import astar
from youbot_control.lib.mecanum import body_to_wheel_speeds, wheel_speeds_to_body
from youbot_control.lib.pure_pursuit import PurePursuit


def test_mecanum_forward_inverse_roundtrip():
    # A body twist -> wheel speeds -> body twist should recover the twist.
    vx, vy, wz = 0.3, -0.15, 0.4
    w1, w2, w3, w4 = body_to_wheel_speeds(vx, vy, wz)
    rvx, rvy, rwz = wheel_speeds_to_body(w1, w2, w3, w4)
    assert abs(rvx - vx) < 1e-9
    assert abs(rvy - vy) < 1e-9
    assert abs(rwz - wz) < 1e-9


def test_mecanum_pure_rotation_has_no_translation():
    w = body_to_wheel_speeds(0.0, 0.0, 0.5)
    vx, vy, wz = wheel_speeds_to_body(*w)
    assert abs(vx) < 1e-9 and abs(vy) < 1e-9
    assert wz > 0.0


def test_astar_finds_path_around_obstacle():
    # 20x20 free grid with a wall down the middle leaving a gap at the top.
    grid = np.zeros((20, 20), dtype=np.uint8)
    grid[5:20, 10] = 1  # vertical wall, open near row 0-4
    path = astar.plan(grid, resolution=0.5, arena_size=10.0,
                      start_world=(-2.5, 0.0), goal_world=(2.5, 0.0))
    assert path, "expected a path around the wall"
    # First and last waypoints should be near start / goal.
    assert abs(path[0][0] - (-2.5)) < 1.0
    assert abs(path[-1][0] - 2.5) < 1.0


def test_pure_pursuit_reports_success_at_goal():
    pp = PurePursuit(position_tolerance=0.15)
    pp.set_path([(0.0, 0.0), (1.0, 0.0)])
    status, *_ = pp.step(1.0, 0.0, 0.0)
    assert status == "success"
