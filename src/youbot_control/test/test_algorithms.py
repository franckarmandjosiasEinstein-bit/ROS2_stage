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
from youbot_control.lib.vision import red_mask, close_mask, blob_centroids


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


# --- perception -------------------------------------------------------------
# The detector went blind for a whole field run because the world's realistic
# rendering added a near-white specular term to the berries, which no
# channel-RATIO test survives. These pin the properties that matter.

def _frame(background, patches):
    img = np.zeros((60, 80, 3), np.uint8)
    img[:] = background
    for (y0, y1, x0, x1), colour in patches:
        img[y0:y1, x0:x1] = colour
    return img


def test_red_mask_survives_a_white_specular_highlight():
    # A lit glossy berry renders as (255, 106, 92): red to the eye, but
    # 106 > 0.33*255, which is what the old ratio test rejected.
    lit = _frame((38, 202, 46), [((20, 30, 20, 30), (255, 106, 92))])
    assert red_mask(lit)[25, 25]
    shaded = _frame((38, 202, 46), [((20, 30, 20, 30), (238, 42, 38))])
    assert red_mask(shaded)[25, 25]


def test_red_mask_rejects_the_robots_own_orange_arm():
    # KUKA orange leads green by 40 but blue by 215; a red surface reflects
    # little of both, so the |g-b| symmetry term separates them.
    arm = _frame((38, 202, 46), [((20, 30, 20, 30), (255, 215, 40))])
    assert not red_mask(arm).any()


def test_closing_keeps_a_stem_split_berry_as_one_fruit():
    yy, xx = np.ogrid[:60, :80]
    img = np.zeros((60, 80, 3), np.uint8)
    img[:] = (38, 202, 46)
    img[(yy - 30) ** 2 + (xx - 40) ** 2 <= 144] = (255, 106, 92)
    img[29:32, 28:53] = (38, 202, 46)          # a 3 px stem across the fruit
    raw = blob_centroids(red_mask(img), min_pixels=6)
    closed = blob_centroids(close_mask(red_mask(img), 2), min_pixels=6,
                            min_fill=0.45, max_aspect=3.0)
    assert len(raw) == 2, "the stem splits the fruit before closing"
    assert len(closed) == 1, "closing must put it back together"
    assert abs(closed[0][0] - 40) < 2 and abs(closed[0][1] - 30) < 2


def test_shape_gates_reject_a_highlight_along_a_gutter_lip():
    # A long thin red streak fills its bounding box completely, so only the
    # aspect gate catches it.
    streak = _frame((38, 202, 46), [((20, 23, 10, 70), (250, 60, 55))])
    mask = close_mask(red_mask(streak), 2)
    assert blob_centroids(mask, min_pixels=6)                      # it is there
    assert not blob_centroids(mask, min_pixels=6, min_fill=0.45, max_aspect=3.0)
