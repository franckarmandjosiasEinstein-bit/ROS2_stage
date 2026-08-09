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
LAUNCH_ARGS=()

# Our own flags are taken out; everything else is a launch argument and is
# passed straight through. Filtering into a second array rather than
# shifting: `shift` inside `for arg in "$@"` removes the FIRST argument each
# time, not the one being looked at, so `./run_sim.sh gui:=false --raw` would
# have dropped gui:=false and handed --raw to ros2 launch.
for arg in "$@"; do
    case "$arg" in
        --raw) PRETTY_ARGS+=(--raw) ;;
        -h|--help) sed -n '2,26p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 0 ;;
        *) LAUNCH_ARGS+=("$arg") ;;
    esac
done

say()  { printf '\033[36m[run_sim]\033[0m %s\n' "$*"; }
warn() { printf '\033[33m[run_sim]\033[0m %s\n' "$*"; }
die()  { printf '\033[31m[run_sim] %s\033[0m\n' "$*" >&2; exit 1; }

# EVERY `source` IN THIS FILE GOES THROUGH HERE.
#
# ROS's setup.bash and the ones colcon generates read variables that they do
# not set -- COLCON_TRACE, AMENT_TRACE_SETUP_FILES, _colcon_prefix_chain... --
# and under `set -u` reading an unset variable kills the shell outright. The
# result is this script dying on line 1 of its real work with
#
#     install/setup.bash: line 11: COLCON_TRACE: unbound variable
#
# which names a file this project does not own and gives no hint that the
# cause is a flag set at the top of the script that is reading it. `set -u`
# is worth keeping for our own code -- it is what turns a typo'd variable
# into an error instead of an empty string -- so it is lifted here and only
# here, around other people's files.
source_ros() {
    set +u
    # shellcheck disable=SC1090
    . "$1"
    set -u
}

# ---------------------------------------------------------- 1. virtualenv
if [ -z "${VIRTUAL_ENV:-}" ]; then
    for cand in "$HOME/.venvs/agri" "$WS/.venv" "$HERE/.venv"; do
        if [ -f "$cand/bin/activate" ]; then
            source_ros "$cand/bin/activate"
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
        [ -f "$setup" ] && { source_ros "$setup"; say "ROS: $setup"; break; }
    done
fi
command -v ros2 >/dev/null 2>&1 || die "ros2 is not on PATH and /opt/ros/jazzy
          was not there either. Source your ROS installation first."

if [ -f "$WS/install/setup.bash" ]; then
    source_ros "$WS/install/setup.bash"
    say "workspace: $WS/install"
else
    warn "no $WS/install/setup.bash -- build first:"
    warn "    cd $WS && colcon build --symlink-install --base-paths cloud_agri/ros2/src"
    die  "nothing to launch"
fi
ros2 pkg prefix agri_robot >/dev/null 2>&1 \
    || die "agri_robot is not in this workspace's install. Build it:
              cd $WS && colcon build --symlink-install --base-paths cloud_agri/ros2/src"

# ---------------------------------------------- 2a. no leftover robot_node
# THE FIGHT THIS PREVENTS.
#
# A robot_node's MQTT client id is "agri-<robot_id>" -- fixed, not per
# process. If an earlier run's robot_node survived (a closed terminal, a
# hard kill during a slow Gazebo load -- Ctrl-C never reaches a process
# that is already dead), it is still sitting on the broker with that id.
# Launching a second one here does not fail: the broker just disconnects
# whichever one it kicks LAST, over and over, which is the "connected" /
# "the broker went away" pair repeating every second or two -- not a
# flaky network, two clients fighting over one identity.
#
# It is worse than noisy. agri/v1/request is BROADCAST, not per-node, so
# for every window both happen to be connected, BOTH act on the same
# order and both drive the one simulated robot's /cmd_vel. That is the
# "it's on the cross, then it moves and ends up between two" symptom:
# not a docking bug, but two uncoordinated controllers pulling the same
# wheels toward two different ideas of where to go.
STALE="$(pgrep -af 'lib/agri_robot/robot_node' 2>/dev/null || true)"
if [ -n "$STALE" ]; then
    warn "a robot_node from an earlier run is still running:"
    printf '%s\n' "$STALE" | sed 's/^/[run_sim]     /'
    die "starting a second one would fight the first for the broker AND
          drive the same robot from two uncoordinated processes at once.
          Stop it first:
              pkill -f 'lib/agri_robot/robot_node'
          then run this again."
fi

# ------------------------------------------------- 2b. the generated world
# The world and the URDF are GENERATED and are not in git. Building them
# here means the answer to "did I remember to run make_plants" is "you
# cannot forget" -- which is not a hypothetical: a pull that aborted on a
# locally-modified world once left a demonstration showing green spheres,
# with nothing on screen saying the plant meshes had never been downloaded.
#
# agri/world/ensure.py decides. It also catches the mesh URI that points at
# another machine's disk, which Gazebo renders as nothing at all and does
# not mention in any log.
if ! ENSURED="$(python3 -m agri.world.ensure 2>&1)"; then
    printf '%s\n' "$ENSURED" >&2
    die "could not generate the world. Run it by hand to see why:
              python3 -m agri.world.ensure"
fi
if [ -n "$ENSURED" ]; then
    while IFS= read -r line; do say "world: $line"; done <<< "$ENSURED"
fi

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

    # robot_node BY NAME, directly: it is this project's own process and
    # Phase B's kill_sim.sh has never heard of it, so a clean SIGINT to
    # $LAUNCH above is the ONLY thing that normally reaps it. That is fine
    # for a Ctrl-C in this window; it does nothing for a hard-killed
    # terminal, which is exactly the case that left one running behind
    # last time and started the fight the next launch guards against
    # (see "2a. no leftover robot_node" above). Belt and suspenders.
    pkill -f 'lib/agri_robot/robot_node' 2>/dev/null && \
        say "stopped a robot_node that outlived the launch"

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

# The ${a[@]+"${a[@]}"} form expands to nothing at all when the array is
# empty, instead of to one empty argument -- and, on bash before 4.4, instead
# of tripping `set -u`. Both arrays are empty in the common case.
exec 3> >(tee -a "$RAWLOG" \
          | python3 "$HERE/tools/prettylog.py" \
                    ${PRETTY_ARGS[@]+"${PRETTY_ARGS[@]}"})
stdbuf -oL -eL ros2 launch agri_robot agri.launch.py \
       ${LAUNCH_ARGS[@]+"${LAUNCH_ARGS[@]}"} >&3 2>&3 &
LAUNCH=$!

wait "$LAUNCH"
STATUS=$?
kill "$WATCHER" 2>/dev/null
exit "$STATUS"
