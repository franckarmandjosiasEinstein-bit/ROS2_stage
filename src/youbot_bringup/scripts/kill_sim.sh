#!/usr/bin/env bash
#
# kill_sim.sh -- stop EVERYTHING the simulation started, including the parts
# that survive Ctrl-C.
#
# WHY THIS FILE EXISTS
#
# `ros2 launch` sends SIGINT to the processes it started, and that is all it
# can do. Three kinds of process are not among them:
#
#   1. GRANDCHILDREN. `gz sim` is launched through ros_gz_sim's own launch
#      file, which runs a ruby wrapper that forks the server and the GUI as
#      separate processes. Launch kills the wrapper; the two it forked keep
#      the world running, keep holding the render context, and keep answering
#      on /world/greenhouse/... so the NEXT run finds a world already there.
#
#   2. THE CAMERA WATCHDOG. The `gui_follow watchdog` helper is an infinite
#      `while sleep 20` shell loop. Launch does start it, so it does get a
#      signal -- but bash in a `sleep` does not act on it until the sleep
#      returns, and any `gz topic -e` child it spawned outlives it regardless.
#
#   3. WHATEVER CRASHED EARLIER and was never in launch's table at all.
#
# So: one script, called from the launch files' OnShutdown handler AND usable
# by hand, that names every pattern once. Duplicating these patterns across
# three launch files is how they drift apart, and a cleanup that misses one
# process is not a cleanup -- the symptom is a second run that behaves oddly
# for reasons that have nothing to do with the code being tested.
#
# USAGE
#     ros2 run youbot_bringup kill_sim.sh        # not installed as an exe
#     bash scripts/kill_sim.sh                   # from the repo
#     bash $(ros2 pkg prefix youbot_bringup)/share/youbot_bringup/scripts/kill_sim.sh
#
#     --quiet   say nothing unless something had to be killed
#     --now     skip the polite pass and SIGKILL straight away. Used by the
#               launch files: by the time OnShutdown fires, launch has already
#               sent SIGINT to everything it owns and waited, so a second
#               polite round would only add two seconds to every Ctrl-C.
#
# SAFETY
#
# Every pattern below is specific to this simulation. There is deliberately no
# `pkill python3`, no `pkill ruby` and no `pkill -f ros2`: those would take out
# the user's editor, an unrelated notebook, or the very shell running this. If
# you add a pattern, make it name something only this project starts.
#
# Note also what is NOT here: `youbot_bringup`. This script lives in that
# package's share directory, so its own command line contains the string --
# a pattern matching it would make the script kill itself half way through.
# The ancestry guard below catches that anyway, but not listing it is the
# first line of defence.

set -u

QUIET=0
NOW=0
for arg in "$@"; do
    case "$arg" in
        --quiet) QUIET=1 ;;
        --now)   NOW=1 ;;
        *) echo "kill_sim: unknown option $arg" >&2; exit 2 ;;
    esac
done

say() { [ "$QUIET" = 1 ] || echo "$@"; }

# Patterns, most specific first. Each is passed to `pkill -f`, so it is matched
# against the full command line.
PATTERNS=(
    ': gui_follow watchdog'      # the camera watchdog loop (its own marker)
    'gz topic -e -t /gui/currently_tracked'   # children it may have left
    'gz sim'
    'gz-sim'
    'ruby .*gz sim'
    'ign gazebo'                 # older installs
    'parameter_bridge'
    'ros_gz_sim.*create'
    'rviz2'
    'rqt_image_view'
    'robot_state_publisher'
    'youbot_control/'            # the control nodes, by their install path
    'youbot_slam/'
    'youbot_gazebo/'
    'youbot_commissioning/'
)

# PIDs that must never be killed: this script and every one of its ancestors.
#
# `pkill -f` matches whole command lines, and a command line can contain a
# pattern by accident. Writing this file's own test the obvious way --
#     bash -c '... start a fake "gz sim" ...; kill_sim.sh; ...'
# -- killed the test, because the wrapper shell's command line contained the
# string "gz sim". The same trap is one careless invocation away for a real
# user: run this from a shell whose command line happens to name one of the
# patterns and the cleanup takes the terminal with it. So the script resolves
# its own ancestry once and refuses to signal anything in it.
SELF=$$
PROTECTED=" $SELF "
_p=$SELF
while :; do
    _p=$(ps -o ppid= -p "$_p" 2>/dev/null | tr -d ' ')
    [ -n "${_p:-}" ] && [ "$_p" != "0" ] || break
    PROTECTED="$PROTECTED$_p "
    [ "$_p" = "1" ] && break
done

victims() {
    # PIDs whose command line matches $1, excluding our own ancestry.
    local pid
    for pid in $(pgrep -f "$1" 2>/dev/null); do
        case "$PROTECTED" in *" $pid "*) continue ;; esac
        echo "$pid"
    done
}

alive() { [ -n "$(victims "$1")" ]; }

slay() {
    # $1 = pattern, $2 = signal
    local pid
    for pid in $(victims "$1"); do
        kill "-$2" "$pid" 2>/dev/null
    done
}

killed=0
for p in "${PATTERNS[@]}"; do
    alive "$p" || continue
    killed=1
    say "  stopping  $p"
    if [ "$NOW" = 1 ]; then
        slay "$p" KILL
    else
        slay "$p" TERM
    fi
done

# Give what was asked politely a moment, then insist. Two seconds is enough for
# an rclpy node to run its shutdown; Gazebo will not manage it and does not
# need to -- the world is not being saved. Skipped under --now, where the
# polite round has already happened upstream.
if [ "$killed" = 1 ] && [ "$NOW" = 0 ]; then
    sleep 2
    for p in "${PATTERNS[@]}"; do
        alive "$p" || continue
        say "  forcing   $p"
        slay "$p" KILL
    done
fi

# Gazebo leaves lock files and stale sockets that make the NEXT run fail with
# an unhelpful message about the world already being loaded. Only when we
# actually stopped something, so running this script while a second, unrelated
# Gazebo is up does not pull the floor out from under it.
[ "$killed" = 1 ] && rm -rf /tmp/gz-* 2>/dev/null

# Report what, if anything, refused to die. Silence here is the point of the
# script; a name here is something new that needs a pattern above.
sleep 0.5
left=""
for p in "${PATTERNS[@]}"; do
    alive "$p" && left="$left  $p"
done
if [ -n "$left" ]; then
    echo "kill_sim: STILL RUNNING after SIGKILL --$left"
    echo "kill_sim: check with  ps -eo pid,cmd | grep -E 'gz sim|youbot_'"
    exit 1
fi

if [ "$killed" = 1 ]; then
    say "kill_sim: the simulation is stopped, nothing left in the background."
else
    say "kill_sim: nothing was running."
fi
exit 0
