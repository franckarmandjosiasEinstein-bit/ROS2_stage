#!/usr/bin/env bash
# run_sim.sh -- ONE command for everything that is not the Cloud.
#
#     ./run_sim.sh                        the greenhouse, the robot, RViz
#     ./run_sim.sh --raw                  do not filter the log
#     ./run_sim.sh gui:=false             any launch argument is passed on
#
# It starts, in this order and for these reasons:
#
#     the virtualenv    every agri.* import lives there. Forgetting it is the
#                       single most repeated mistake in this project: the node
#                       dies on `No module named 'qrcode'` one second after
#                       Gazebo opens, the window looks perfect, and every
#                       request issued afterwards goes nowhere.
#     ROS 2 + the ws    /opt/ros/jazzy and this workspace's install/
#     mosquitto         BEFORE the launch, because the robot node connects at
#                       startup. It now retries rather than dying, so the
#                       order is no longer load-bearing -- but a broker that
#                       is already there means no error to explain.
#     the launch        Gazebo, robot_state_publisher, the bridge, agri_viz,
#                       RViz and the robot node
#
# and it stops all of it -- including the parts that survive Ctrl-C -- on
# Ctrl-C, or when the Cloud asks (see agri/session.py).

set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname "$HERE")"                       # the colcon workspace above us
STOP="${XDG_RUNTIME_DIR:-/tmp}/agri-sim-${USER:-agri}.stop"
LOGDIR="$HERE/logs"
RAWLOG="$LOGDIR/sim-$(date +%Y%m%d-%H%M%S).log"
PRETTY_ARGS=()

for arg in "$@"; do
    case "$arg" in
        --raw) PRETTY_ARGS+=(--raw); shift ;;
        -h|--help) sed -n '2,26p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 0 ;;
    esac
done

say()  { printf '\033[36m[run_sim]\033[0m %s\n' "$*"; }
warn() { printf '\033[33m[run_sim]\033[0m %s\n' "$*"; }
die()  { printf '\033[31m[run_sim] %s\033[0m\n' "$*" >&2; exit 1; }

# ---------------------------------------------------------- 1. virtualenv
if [ -z "${VIRTUAL_ENV:-}" ]; then
    for cand in "$HOME/.venvs/agri" "$WS/.venv" "$HERE/.venv"; do
        if [ -f "$cand/bin/activate" ]; then
            # shellcheck disable=SC1091
            . "$cand/bin/activate"
            say "virtualenv: $cand"
            break
        fi
    done
fi
[ -n "${VIRTUAL_ENV:-}" ] || die "no virtualenv found (looked in ~/.venvs/agri,
          $WS/.venv, $HERE/.venv). Activate yours and run this again:
              source ~/.venvs/agri/bin/activate"
python3 -c 'import qrcode, paho.mqtt.client, cryptography' 2>/dev/null \
    || die "the virtualenv at $VIRTUAL_ENV is missing dependencies. Install:
              pip install -e $HERE"

# --------------------------------------------------------------- 2. ROS 2
if ! command -v ros2 >/dev/null 2>&1; then
    for setup in /opt/ros/jazzy/setup.bash /opt/ros/humble/setup.bash; do
        # shellcheck disable=SC1090
        [ -f "$setup" ] && { . "$setup"; say "ROS: $setup"; break; }
    done
fi
command -v ros2 >/dev/null 2>&1 || die "ros2 is not on PATH and /opt/ros/jazzy
          was not there either. Source your ROS installation first."

if [ -f "$WS/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    . "$WS/install/setup.bash"
    say "workspace: $WS/install"
else
    warn "no $WS/install/setup.bash -- build first:"
    warn "    cd $WS && colcon build --symlink-install --base-paths cloud_agri/ros2/src"
    die  "nothing to launch"
fi
ros2 pkg prefix agri_robot >/dev/null 2>&1 \
    || die "agri_robot is not in this workspace's install. Build it:
              cd $WS && colcon build --symlink-install --base-paths cloud_agri/ros2/src"

# ----------------------------------------------------------- 3. the broker
# /dev/tcp is a bash builtin, so this needs no nc, no lsof and no ss.
broker_up() { (exec 3<>/dev/tcp/localhost/1883) 2>/dev/null; }

MOSQ=""
if broker_up; then
    say "broker: already running on localhost:1883"
elif command -v mosquitto >/dev/null 2>&1; then
    mkdir -p "$LOGDIR"
    mosquitto -p 1883 >"$LOGDIR/mosquitto.log" 2>&1 &
    MOSQ=$!
    for _ in 1 2 3 4 5 6 7 8 9 10; do broker_up && break; sleep 0.3; done
    if broker_up; then
        say "broker: started mosquitto on localhost:1883 (pid $MOSQ)"
    else
        warn "mosquitto did not come up; see $LOGDIR/mosquitto.log"
        MOSQ=""
    fi
else
    warn "mosquitto is not installed, so nothing can order a measurement."
    warn "    sudo apt install mosquitto mosquitto-clients"
    warn "Continuing: the simulation runs, the robot will wait for a broker."
fi

# ------------------------------------------------------------- 4. shutdown
CLEANED=0
cleanup() {
    [ "$CLEANED" = 1 ] && return
    CLEANED=1
    echo
    say "stopping the simulation"

    # Polite first: launch forwards SIGINT to everything it owns.
    if [ -n "${LAUNCH:-}" ] && kill -0 "$LAUNCH" 2>/dev/null; then
        kill -INT "$LAUNCH" 2>/dev/null
        for _ in $(seq 1 20); do
            kill -0 "$LAUNCH" 2>/dev/null || break
            sleep 0.5
        done
        kill -0 "$LAUNCH" 2>/dev/null && kill -KILL "$LAUNCH" 2>/dev/null
    fi

    # Then the ones launch cannot reach: gz forks the server and the GUI
    # through a ruby wrapper, and the camera watchdog sits in `sleep 20`.
    # Phase B already owns that list; reusing it means one place to fix.
    local killer=""
    if command -v ros2 >/dev/null 2>&1; then
        local prefix
        prefix="$(ros2 pkg prefix youbot_bringup 2>/dev/null)"
        [ -n "$prefix" ] && killer="$prefix/share/youbot_bringup/scripts/kill_sim.sh"
    fi
    [ -f "${killer:-}" ] || killer="$WS/src/youbot_bringup/scripts/kill_sim.sh"
    if [ -f "$killer" ]; then
        bash "$killer" --now --quiet
        say "cleaned up with kill_sim.sh"
    else
        warn "kill_sim.sh not found; Gazebo may have left processes behind"
        warn "    check with:  pgrep -af 'gz sim'"
    fi

    if [ -n "$MOSQ" ] && kill -0 "$MOSQ" 2>/dev/null; then
        kill -TERM "$MOSQ" 2>/dev/null
        say "stopped the broker we started (pid $MOSQ)"
    fi

    rm -f "$STOP"
    exec 3>&- 2>/dev/null
    sleep 0.4                      # let the log filter drain its last lines
    say "full log: $RAWLOG"
}
trap cleanup EXIT INT TERM

# -------------------------------------------------- 5. the Cloud's request
# A stale file from a SIGKILLed run would stop this one before it started.
rm -f "$STOP"
(
    while sleep 1; do
        kill -0 "$$" 2>/dev/null || exit 0
        if [ -e "$STOP" ]; then
            printf '\033[36m[run_sim]\033[0m %s\n' \
                   "the Cloud asked for a full stop"
            kill -INT "$$" 2>/dev/null
            exit 0
        fi
    done
) &
WATCHER=$!

# --------------------------------------------------------- 6. the launch
mkdir -p "$LOGDIR"
say "log: $RAWLOG"
say "stop with Ctrl-C here, or with QUITTER on the dashboard"
echo

exec 3> >(tee -a "$RAWLOG" | python3 "$HERE/tools/prettylog.py" "${PRETTY_ARGS[@]}")
stdbuf -oL -eL ros2 launch agri_robot agri.launch.py "$@" >&3 2>&3 &
LAUNCH=$!

wait "$LAUNCH"
STATUS=$?
kill "$WATCHER" 2>/dev/null
exit "$STATUS"
