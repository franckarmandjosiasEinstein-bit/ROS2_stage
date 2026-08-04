#!/usr/bin/env bash
#
# Repo-root escape hatch. The real script lives in youbot_bringup, because the
# launch files call it on shutdown and they can only reach the install space.
# This wrapper just means you do not have to remember that path when something
# has been left running and you are standing in the repo.
#
#     bash scripts/kill_sim.sh          # stop everything, politely then not
#     bash scripts/kill_sim.sh --now    # straight to SIGKILL
#
# Shutdown normally handles this by itself: every launch file runs the same
# script when you Ctrl-C. This is for the times it did not get the chance --
# a closed terminal, a killed launch, a crash.

set -u
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
real="$here/../src/youbot_bringup/scripts/kill_sim.sh"

if [ ! -f "$real" ]; then
    # Fall back to the installed copy, in case this wrapper was copied out of
    # the repo on its own.
    real=$(ros2 pkg prefix youbot_bringup 2>/dev/null)/share/youbot_bringup/scripts/kill_sim.sh
fi
if [ ! -f "$real" ]; then
    echo "kill_sim: cannot find the real script (looked in src/ and in the" >&2
    echo "          youbot_bringup install space). Is the workspace built?" >&2
    exit 1
fi

exec bash "$real" "$@"
