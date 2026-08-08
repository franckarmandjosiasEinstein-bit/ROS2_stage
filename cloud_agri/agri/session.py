"""session -- how the Cloud and the simulation stop each other.

Two commands start this system, and neither is the other's parent:

    ./run_sim.sh      the broker, Gazebo, RViz and the robot node
    ./run_cloud.sh    the Cloud: its console, its dashboard, its keys

That separation is deliberate -- the Cloud is meant to be able to sit on
another computer, which is the whole point of putting a broker between them
-- so neither process can stop the other with a signal, and neither has the
other's process group.

They agree on ONE FILE instead. Stopping the Cloud, whether by Ctrl-C in its
console or by pressing QUITTER on the dashboard, creates it. run_sim.sh
watches for it and takes the simulation down when it appears. run_sim.sh also
removes it at startup and at exit, so a stale one can only survive a SIGKILL,
and the next run clears it before anything reads it.

WHY A FILE AND NOT A PID

A PID can be reused between the moment it is written down and the moment
somebody kills it, and the process on the other end of a reused PID belongs
to someone else. A file cannot kill the wrong thing.

It also fails in the right direction for the deployment this system is
designed around. With the Cloud on a second machine the file is created on
THAT machine's disk, nobody is watching for it, and the simulation keeps
running -- which is correct: a Cloud on another computer has no business
killing a robot, and the operator standing next to the robot should be the
one who stops it.

WHERE THE FILE GOES

$XDG_RUNTIME_DIR when there is one (a per-user tmpfs, cleared at logout),
/tmp otherwise, and the user's name is in the filename either way so two
people on one machine cannot stop each other's simulation. run_sim.sh spells
the same path in shell; check_cloud.py asserts the two spellings agree,
because a contract written down twice is a contract that drifts.
"""

from __future__ import annotations

import os
from pathlib import Path

#: Kept as a literal so the shell script and this module can be compared
#: character for character by the test suite.
STOP_BASENAME = "agri-sim-{user}.stop"


def runtime_dir() -> Path:
    d = os.environ.get("XDG_RUNTIME_DIR")
    return Path(d) if d and Path(d).is_dir() else Path("/tmp")


def stop_file() -> Path:
    """The file whose EXISTENCE means "take the simulation down"."""
    return runtime_dir() / STOP_BASENAME.format(
        user=os.environ.get("USER") or "agri")


def ask_simulation_to_stop() -> Path | None:
    """Create it. Returns the path, or None if it could not be written.

    Failure is not an error worth propagating: the Cloud is shutting down,
    and a Cloud that could not write a hint file has still done its own job.
    The caller says what happened and carries on.
    """
    path = stop_file()
    try:
        path.write_text("stop\n")
        return path
    except OSError:
        return None


def clear_stop_request() -> None:
    """Called by run_sim.sh's Python side of the contract, and by the tests."""
    try:
        stop_file().unlink()
    except OSError:
        pass
