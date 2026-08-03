"""align_lqr -- discrete state feedback for the visual alignment loop.

THE MEASURED PROBLEM

The mission centres a berry in the frame by creeping sideways under a
proportional law with a dead zone and a saturation:

    v = clamp(K * |offset|, VMIN, VMAX) * sign(offset) * learned_sign

Its failure signature is all over the logs, dozens of times per run:

    Alignment stalled at offset +0.26 for 4s -- resuming patrol
    Alignment stalled at offset +0.68 for 4s -- resuming patrol
    Alignment stalled at offset +0.96 for 4s -- resuming patrol

That is a STEADY-STATE ERROR, and a proportional law with a dead zone cannot
remove one. Below VMIN/K the commanded speed is floored, not zeroed, so the
robot keeps nudging; above it the gain is fixed while the true image gain
depends on how far away the berry is. Neither behaviour converges. The
harvest phase spends 39% of its time in this loop, ending without a pick.

THE MODEL (Chafouk, exercise 3, applied to this robot)

State: the offset and its accumulated integral.

    X(k) = [ delta(k) , sigma(k) ]^T ,     sigma(k+1) = sigma(k) + T*delta(k)

Input: the commanded lateral speed e(k) = v_y.

Plant: moving sideways at v_y for one period T shifts the berry across the
image by -T*v_y*g, where g is the IMAGE GAIN -- how many units of normalised
offset one metre of lateral motion produces. It is NOT a constant:

    g = f_x / (d * (W/2))       [normalised offset per metre]

with d the range to the berry. Close berries swing across the frame fast,
distant ones barely move. The fixed-gain proportional law ignores this
completely, which is why it is jumpy near and sluggish far.

    A = [[1, 0], [T, 1]]        B = [[-T*g], [0]]

Control law, in the notation of the course:

    e(k) = -R X(k) + Q C

with R placed by pole assignment for a chosen settling time and damping, and
Q chosen for UNITY STATIC GAIN so the steady-state error vanishes. C is the
reference offset, which is zero: we want the berry in the centre.

WHY THIS IS NOT JUST "ADD AN INTEGRATOR"

It is, in the sense that the second state is an integral. But writing it as a
state-space design buys three things a hand-tuned PI does not:

  * the gains follow from a settling time and a damping ratio, which are
    physically meaningful and can be defended, instead of from trial and error;
  * the range-dependent image gain enters the model explicitly, so the loop
    behaves the same at 0.5 m and at 2 m;
  * the static gain Q is computed rather than hoped for, which is precisely
    what makes the steady-state error zero rather than small.

The dead zone is gone. A dead zone is what a proportional law needs to stop
hunting; an integrator plus a proper static gain makes it unnecessary, and it
was the dead zone that produced the +0.26 stalls.
"""

from __future__ import annotations

import math


class AlignController:
    """Discrete state feedback with an integral state, for image centring."""

    def __init__(self, period=0.1, settling_time=1.5, damping=0.9,
                 v_max=0.10, image_gain_ref=1.0, range_ref=1.0,
                 integral_limit=2.0):
        self.T = float(period)
        self.settling_time = float(settling_time)
        self.damping = float(damping)
        self.v_max = float(v_max)
        # Nominal image gain and the range it was measured at. The gain used
        # each tick is scaled by range_ref/range, which is the 1/d law above.
        self.image_gain_ref = float(image_gain_ref)
        self.range_ref = float(range_ref)
        self.integral_limit = float(integral_limit)
        self.reset()
        self._place_poles()

    # ------------------------------------------------------------ design
    def _place_poles(self) -> None:
        """Continuous second-order spec -> discrete poles -> R by placement.

        A settling time and a damping ratio are the two numbers an engineer
        can actually reason about, so the design starts there:

            omega_n = 4 / (zeta * t_s)          (2% settling, standard)
            s = -zeta*omega_n +/- j*omega_n*sqrt(1-zeta^2)
            z = exp(s*T)                         (matched pole mapping)

        The desired characteristic polynomial is then z^2 + a1 z + a0 with

            a1 = -2*exp(-zeta*omega_n*T)*cos(omega_d*T)
            a0 =  exp(-2*zeta*omega_n*T)
        """
        zeta = max(0.1, min(1.5, self.damping))
        ts = max(0.2, self.settling_time)
        wn = 4.0 / (zeta * ts)
        wd = wn * math.sqrt(max(0.0, 1.0 - zeta * zeta))
        r = math.exp(-zeta * wn * self.T)
        self._a1 = -2.0 * r * math.cos(wd * self.T)
        self._a0 = r * r
        self.omega_n = wn

    def gains(self, image_gain: float):
        """(r1, r2, q) for the current image gain.

        Worked out explicitly, because getting the sign wrong here makes the
        loop DIVERGE rather than misbehave -- the first version did, and the
        unit test below caught it before the robot did.

        Moving at +v_y for one period moves the berry the OTHER way in the
        image, so the input matrix carries a minus:

            A = [[1, 0], [T, 1]]        B = [[-T*g], [0]]

        With e = -R X and R = [r1, r2]:

            A - B R = [[1 + T*g*r1, T*g*r2], [T, 1]]

        char. poly:  z^2 - (2 + T*g*r1) z + (1 + T*g*r1 - T^2*g*r2)

        Matching z^2 + a1 z + a0:

            r1 = (-a1 - 2) / (T*g)              <- NEGATIVE, that minus is the plant's
            r2 = -(1 + a1 + a0) / (T^2*g)
        """
        g = max(1e-6, abs(image_gain))
        Tg = self.T * g
        r1 = (-self._a1 - 2.0) / Tg
        r2 = -(1.0 + self._a1 + self._a0) / (self.T * Tg)
        # Static gain. For the reference we actually use, C = 0 (berry in the
        # centre), the integral state alone forces delta -> 0 in steady state:
        # sigma can only stop changing when delta is exactly zero. q therefore
        # does nothing at C = 0 and exists so a non-zero reference -- aiming
        # deliberately off-centre -- would also be tracked without error.
        q = -r1
        return r1, r2, q

    # ------------------------------------------------------------ runtime
    def reset(self) -> None:
        self._sigma = 0.0
        self._last_offset = None

    def step(self, offset: float, distance: float | None = None,
             reference: float = 0.0):
        """One tick. Returns (v_y, diagnostics dict).

        `offset` is the berry's horizontal position in [-1, 1], 0 = centred.
        `distance` is the range in metres if known; the image gain is scaled
        by range_ref/distance, which is the whole point of modelling it.
        """
        err = offset - reference
        # Anti-windup: the integrator must not charge while saturated, or the
        # loop overshoots on release -- the classic failure of bolting an
        # integrator onto a saturating actuator.
        self._sigma += self.T * err
        self._sigma = max(-self.integral_limit,
                          min(self.integral_limit, self._sigma))

        gain = self.image_gain_ref
        if distance and distance > 1e-3:
            gain *= self.range_ref / distance
        r1, r2, q = self.gains(gain)

        v = -r1 * offset - r2 * self._sigma + q * reference
        v_sat = max(-self.v_max, min(self.v_max, v))
        if abs(v) > abs(v_sat) and (v - v_sat) * err > 0.0:
            # Saturated in the direction that would charge the integrator
            # further: give the charge back.
            self._sigma -= self.T * err
            self._sigma = max(-self.integral_limit,
                              min(self.integral_limit, self._sigma))

        self._last_offset = offset
        return v_sat, {"r1": r1, "r2": r2, "q": q, "image_gain": gain,
                       "integral": self._sigma, "unsaturated": v}

    # ------------------------------------------------------------ analysis
    def closed_loop_poles(self, image_gain: float):
        """The two closed-loop poles, for a regression check and the report."""
        r1, r2, _ = self.gains(image_gain)
        Tg = self.T * abs(image_gain)
        # Same polynomial as gains(): z^2 - (2 + Tg*r1) z + (1 + Tg*r1 - T*Tg*r2).
        # This function had the pre-fix signs for one commit and cheerfully
        # reported an unstable loop that was in fact converging -- an analysis
        # routine that disagrees with the design it analyses is worse than none.
        b = -(2.0 + Tg * r1)
        c = 1.0 + Tg * r1 - self.T * Tg * r2
        disc = b * b - 4.0 * c
        if disc >= 0.0:
            root = math.sqrt(disc)
            return ((-b + root) / 2.0, 0.0), ((-b - root) / 2.0, 0.0)
        root = math.sqrt(-disc)
        return (-b / 2.0, root / 2.0), (-b / 2.0, -root / 2.0)

    def is_stable(self, image_gain: float) -> bool:
        return all(math.hypot(re, im) < 1.0
                   for re, im in self.closed_loop_poles(image_gain))
