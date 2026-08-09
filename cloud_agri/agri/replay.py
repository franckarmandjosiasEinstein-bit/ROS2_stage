"""replay -- a signed order must stop being an order.

THE HOLE THIS CLOSES

A request is signed and not encrypted, deliberately: "measure P2,5R" is not
a secret, but issuing it drives a robot around a greenhouse. Signing gets
authenticity, which is the property that matters -- and on its own it gets
only half of it.

A signature says WHO wrote a message. It says nothing about WHEN, or about
whether you are reading it for the first time. So anyone who can subscribe
to `agri/v1/request` -- which, on a broker with no ACL, is anyone who can
reach the broker -- can record one genuine order and publish the identical
bytes back a week later. Every check passes: the schema is right, the
targets are real, the signature verifies against the Cloud's public key,
because it IS the Cloud's signature. The robot drives off.

That is not a theoretical attack. It is a copy and a paste.

THE TWO HALVES, AND WHY NEITHER WORKS ALONE

    freshness   the order carries the time it was signed, and the robot
                refuses one older than a window. Bounds how long a captured
                order stays dangerous -- to the window, from forever.

    a seen-set  the robot remembers the ids it has already acted on and
                refuses a repeat. Kills the replay outright.

Freshness alone leaves a window: capture an order and replay it within the
window and it still runs. The seen-set alone would have to remember every
id the robot has ever seen, forever, in RAM, across restarts -- and a robot
that reboots forgets, which is exactly when an attacker would replay.

Together they need no persistence and have no window. The set only has to
remember ids from within the freshness window, because anything older is
already refused on age. Bound the set generously, expire by time, done.

THE REQUEST ID IS THE NONCE

There is no separate `nonce` field, on purpose. `request_id` is already
random and already unique per order, already inside the signature, and
already the thing the whole system uses to correlate an order with its acks
and its reports. A second random number doing the same job would be one
more field to explain and one more thing to keep consistent.

It is 48 bits. Two genuine requests colliding over a decade of daily
campaigns is a 1-in-4000 event, and its consequence is one refused order
that the operator re-issues -- not a wrong measurement. That is the right
side of the trade.

THE CLOCK, WHICH IS THE PART THAT WILL ACTUALLY BITE

Freshness compares two clocks on two machines. If they disagree by more
than the window, every order is refused and the symptom -- a robot that
acknowledges nothing -- looks exactly like a broken broker or a bad key.

So this module does not simply say no. It says how far apart the clocks
are, in which direction, and what to type. An order dated in the FUTURE is
reported differently from a stale one, because a future order cannot be a
replay: it is a clock, every time. And an order that is accepted while
already using more than half the window logs a warning, so a drift that is
growing is visible before the day it starts refusing work.
"""

from __future__ import annotations

from collections import OrderedDict
from datetime import datetime, timezone

TIME_FMT = "%Y-%m-%dT%H:%M:%SZ"

#: How old a request may be and still be obeyed.
#:
#: Two minutes is chosen against the human loop, not the network: an
#: operator types a target, the Cloud signs it, the broker delivers it in
#: milliseconds. Anything that takes minutes to arrive is not a late
#: request, it is a replayed one -- or the robot was offline, in which case
#: re-issuing is the correct thing to do rather than obeying a stale order
#: whose greenhouse has moved on.
FRESHNESS_S = 120.0

#: How many ids to remember. Only ids from inside the window can matter, so
#: this only has to exceed the number of requests the Cloud could sign in
#: two minutes. 512 is far past that and costs a few kilobytes.
REMEMBER = 512


class ReplayError(Exception):
    """A request that verified cryptographically and must still be refused."""


def parse_stamp(text: str) -> datetime:
    """One of our own UTC stamps -> an aware datetime, or raise."""
    try:
        return datetime.strptime(text, TIME_FMT).replace(tzinfo=timezone.utc)
    except (TypeError, ValueError) as exc:
        raise ReplayError(
            f"issued_at {text!r} is not a UTC stamp -- expected "
            f"{TIME_FMT!r}. A request with no usable date cannot be checked "
            "for freshness, and an unchecked one is a replayable one.") from exc


class ReplayGuard:
    """Refuses a request that is stale, or that has been seen already.

    One instance per node, consulted after the signature verifies and
    before the mission is accepted. Not thread-safe by design: the MQTT
    client delivers messages on one thread, and a lock here would be a lock
    nothing contends for.
    """

    def __init__(self, window_s: float = FRESHNESS_S,
                 remember: int = REMEMBER) -> None:
        self.window_s = float(window_s)
        self.remember = int(remember)
        #: id -> the moment we accepted it, oldest first.
        self._seen: OrderedDict[str, datetime] = OrderedDict()
        #: The worst skew observed, for the diagnostic in the log.
        self.max_skew_s = 0.0

    # ------------------------------------------------------------- checking
    def check(self, request_id: str, issued_at: str,
              now: datetime | None = None) -> float:
        """Accept the request and return its age, or raise ReplayError.

        Returns the AGE in seconds -- signed, so a negative age means the
        order is dated in the future. The caller logs it; a caller that
        ignores it still gets the protection.
        """
        now = now or datetime.now(timezone.utc)
        issued = parse_stamp(issued_at)
        age = (now - issued).total_seconds()
        self.max_skew_s = max(self.max_skew_s, abs(age))

        self._forget_old(now)

        # A future-dated order cannot be a replay: nobody can capture a
        # message before it is sent. It is always a clock, so say that
        # rather than accusing the network of an attack.
        if age < -self.window_s:
            raise ReplayError(
                f"request {request_id} is dated {-age:.0f} s in the FUTURE. "
                f"That is not a replay -- it is a clock. This machine and "
                f"the Cloud disagree by {-age:.0f} s; the window is "
                f"{self.window_s:.0f} s.\n"
                f"    Check both:  timedatectl status\n"
                f"    Or widen it: --replay-window {abs(age) * 2:.0f}")

        if age > self.window_s:
            raise ReplayError(
                f"request {request_id} was signed {age:.0f} s ago and the "
                f"window is {self.window_s:.0f} s -- REFUSED.\n"
                f"    If this order is genuine, the two clocks disagree by "
                f"{age:.0f} s (timedatectl status), or the robot was offline "
                f"when it was issued -- re-issue it rather than obeying it "
                f"late.\n"
                f"    If it is not, something just replayed a captured "
                f"order and this is the refusal that stopped it.")

        if request_id in self._seen:
            first = self._seen[request_id]
            raise ReplayError(
                f"request {request_id} was already accepted at "
                f"{first.strftime(TIME_FMT)} -- REFUSED as a replay. A "
                f"signature proves who wrote an order, not that you are "
                f"reading it for the first time.")

        self._seen[request_id] = now
        while len(self._seen) > self.remember:
            self._seen.popitem(last=False)
        return age

    def suspicious_skew(self, age: float) -> str:
        """A warning for an ACCEPTED request whose age eats the window.

        Empty when the age is comfortable. A drift that is growing should
        be visible in the log on the day it is harmless, not on the day it
        stops the robot.
        """
        if abs(age) < self.window_s / 2:
            return ""
        return (f"clock skew {age:+.0f} s against a {self.window_s:.0f} s "
                f"window -- accepted, but this will start refusing orders. "
                f"Sync the clocks (timedatectl set-ntp true).")

    # -------------------------------------------------------------- pruning
    def _forget_old(self, now: datetime) -> None:
        """Drop ids that are older than the window.

        They cannot be replayed any more -- the freshness check refuses them
        on age before the set is even consulted -- so remembering them buys
        nothing and would grow without bound on a long campaign.
        """
        cutoff = self.window_s
        for rid in [r for r, t in self._seen.items()
                    if (now - t).total_seconds() > cutoff]:
            del self._seen[rid]

    @property
    def remembered(self) -> int:
        return len(self._seen)
