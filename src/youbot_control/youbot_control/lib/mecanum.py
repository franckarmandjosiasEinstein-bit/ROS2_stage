"""Mecanum kinematics for the KUKA YouBot base (framework-independent).

Ported verbatim from the validated Webots controller. Keeping it free of
any ROS or Webots import means we can unit-test it on a laptop with plain
`pytest`, and reuse it from both the ROS 2 nodes and the Webots driver.

Body frame convention (REP-103, right-handed):
    vx = forward (+X), vy = left (+Y), wz = yaw CCW (+Z).

Official Cyberbotics base.c mapping, keyed by native wheel index, so no
physical-corner remapping is needed:
    wheel1 = (vx + vy + L*wz) / r      (front-left)
    wheel2 = (vx - vy - L*wz) / r      (front-right)
    wheel3 = (vx - vy + L*wz) / r      (rear-left)
    wheel4 = (vx + vy - L*wz) / r      (rear-right)
where L = LX + LY and r is the wheel radius.
"""

from __future__ import annotations

# YouBot geometry (from the Cyberbotics reference base.c).
WHEEL_RADIUS = 0.05
LX = 0.228          # longitudinal COM-to-wheel distance (m)
LY = 0.158          # lateral COM-to-wheel distance (m)
L_FACTOR = LX + LY  # 0.386
MAX_WHEEL_SPEED = 14.0  # rad/s, safe below the ~14.8 rad/s device ceiling


def body_to_wheel_speeds(vx: float, vy: float, wz: float) -> list[float]:
    """Twist (vx, vy, wz) -> [w1, w2, w3, w4] wheel angular speeds (rad/s).

    Uniformly scaled down if any wheel would exceed MAX_WHEEL_SPEED, so the
    commanded direction of motion is preserved.
    """
    r = WHEEL_RADIUS
    L = L_FACTOR
    speeds = [
        (vx + vy + L * wz) / r,  # wheel1
        (vx - vy - L * wz) / r,  # wheel2
        (vx - vy + L * wz) / r,  # wheel3
        (vx + vy - L * wz) / r,  # wheel4
    ]
    peak = max(abs(s) for s in speeds)
    if peak > MAX_WHEEL_SPEED:
        scale = MAX_WHEEL_SPEED / peak
        speeds = [s * scale for s in speeds]
    return speeds


def wheel_speeds_to_body(w1: float, w2: float, w3: float, w4: float) -> tuple[float, float, float]:
    """Forward kinematics (inverse of the above): wheel speeds -> (vx, vy, wz).

    Useful for wheel odometry: feed the per-step wheel angular deltas and
    integrate the returned body twist.
    """
    r = WHEEL_RADIUS
    L = L_FACTOR
    vx = r * (w1 + w2 + w3 + w4) / 4.0
    vy = r * (w1 - w2 - w3 + w4) / 4.0
    wz = r * (w1 - w2 + w3 - w4) / (4.0 * L)
    return vx, vy, wz
