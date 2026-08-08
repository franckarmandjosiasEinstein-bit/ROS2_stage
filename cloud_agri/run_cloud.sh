#!/usr/bin/env bash
# run_cloud.sh -- ONE command for the Cloud: console, dashboard, keys.
#
#     ./run_cloud.sh                      the normal case
#     ./run_cloud.sh --keep-sim           do not stop the simulation on exit
#     ./run_cloud.sh --broker 192.168.1.7 the Cloud on a second machine
#
# The second of the two commands. run_sim.sh is the first, and this one is
# separate ON PURPOSE: the Cloud is meant to be able to live on another
# computer, which is the whole reason there is a broker between them. Starting
# it from the robot's launch file would make that quietly untrue.
#
# Ctrl-C here, and QUITTER on the dashboard, both export the two CSVs and then
# take the simulation down with them (agri/session.py explains how, and
# --keep-sim opts out).

set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

say()  { printf '\033[36m[run_cloud]\033[0m %s\n' "$*"; }
warn() { printf '\033[33m[run_cloud]\033[0m %s\n' "$*"; }
die()  { printf '\033[31m[run_cloud] %s\033[0m\n' "$*" >&2; exit 1; }

case "${1:-}" in
    -h|--help) sed -n '2,18p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 0 ;;
esac

# ---------------------------------------------------------- 1. virtualenv
if [ -z "${VIRTUAL_ENV:-}" ]; then
    for cand in "$HOME/.venvs/agri" "$(dirname "$HERE")/.venv" "$HERE/.venv"; do
        if [ -f "$cand/bin/activate" ]; then
            # shellcheck disable=SC1091
            . "$cand/bin/activate"
            say "virtualenv: $cand"
            break
        fi
    done
fi
[ -n "${VIRTUAL_ENV:-}" ] || die "no virtualenv found (looked in ~/.venvs/agri).
          source ~/.venvs/agri/bin/activate"

# ------------------------------------------------------------ 2. broker
broker_up() { (exec 3<>/dev/tcp/localhost/1883) 2>/dev/null; }
if ! broker_up && ! printf '%s\n' "$@" | grep -q -- "--broker"; then
    warn "nothing is listening on localhost:1883."
    warn "Start the simulation first -- it brings the broker up:"
    warn "    ./run_sim.sh"
    warn "Waiting up to 30 s for one to appear…"
    for _ in $(seq 1 60); do broker_up && break; sleep 0.5; done
    broker_up || die "still no broker. Start one:  mosquitto -p 1883 -v"
    say "broker is up"
fi

# ------------------------------------------------------------- 3. the Cloud
# Run from cloud_agri/ so keys/ and store/ land beside the code, which is
# where the README, the tests and every previous run already expect them.
cd "$HERE" || die "cannot enter $HERE"
say "keys: $HERE/keys      store: $HERE/store"
echo
exec python3 -m agri.cloud.server --keys keys --store store "$@"
