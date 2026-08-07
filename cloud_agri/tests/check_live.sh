#!/usr/bin/env bash
# check_live.sh -- why is RViz empty? Answered in one command.
#
# Run this in a SECOND terminal while `ros2 launch agri_robot agri.launch.py`
# is running in the first. It reports, in order, the four things that have to
# be true for RViz to draw anything, and stops at the first one that is not.
#
# WHY THIS EXISTS
#
# Because "RViz is empty" has five causes that look identical from the
# window: the launch is not running, agri_viz died, agri_viz is alive but
# never hears /odom, the fixed frame has no transform, or the displays
# themselves are misconfigured. Diagnosing it by hand means knowing to run
# `ros2 topic hz` WHILE the launch is up -- and running it a minute after
# Ctrl-C reports "frame map does not exist", which is true, useless, and
# indistinguishable from the real fault. That mistake cost two rounds, so
# the first thing this script does is refuse to answer when nothing is
# running, instead of answering something misleading.

set -u

green() { printf '\033[32m  ok  \033[0m%s\n' "$1"; }
bad()   { printf '\033[31m FAIL \033[0m%s\n' "$1"; }
info()  { printf '\033[90m      %s\033[0m\n' "$1"; }
rule()  { printf '\n\033[36m%s\033[0m\n' "$1"; }

if ! command -v ros2 >/dev/null 2>&1; then
    bad "ros2 is not on PATH"
    info "source /opt/ros/jazzy/setup.bash && source install/setup.bash"
    exit 1
fi

rule "1. is the launch actually running?"
nodes=$(ros2 node list 2>/dev/null)
if ! grep -q '/agri_viz' <<<"$nodes"; then
    bad "agri_viz is not running, so nothing below can be measured"
    info "Nodes visible right now:"
    if [ -z "$nodes" ]; then
        info "  (none at all)"
    else
        sed 's/^/        /' <<<"$nodes"
    fi
    echo
    info "If you have not started the launch, or you stopped it with Ctrl-C,"
    info "start it and LEAVE IT RUNNING, then run this again:"
    info "    ros2 launch agri_robot agri.launch.py"
    echo
    info "If the launch IS running in another terminal, agri_viz died at"
    info "startup -- look in that window for a traceback from viz_node."
    exit 1
fi
green "agri_viz is up"
for n in gz_bridge robot_state_publisher rviz2; do
    if grep -q "/$n" <<<"$nodes"; then green "$n is up"
    else bad "$n is NOT running"; fi
done

got() {   # got <topic> <seconds> -- did one message arrive?
    timeout "$2" ros2 topic echo "$1" --once >/dev/null 2>&1
}

rule "2. does the odometry reach it? (no /odom, no fixed frame, no picture)"
if got /odom 8; then
    green "/odom is publishing"
else
    bad "/odom delivered nothing in 8 s -- THIS is why RViz is empty"
    info "agri_viz publishes map -> base_link only when odometry arrives."
    info "Check the bridge and that Gazebo is not paused (Play in gz)."
    info "    ros2 topic hz /odom"
    exit 1
fi

rule "3. is the transform RViz needs being published?"
if got /tf 8; then
    green "/tf is publishing"
else
    bad "/tf delivered nothing in 8 s"
fi
if timeout 8 ros2 run tf2_ros tf2_echo map base_link 2>&1 |
        grep -q 'Translation'; then
    green "map -> base_link resolves"
else
    bad "map -> base_link does NOT resolve"
    info "RViz greys every display out when its Fixed Frame has no transform."
    exit 1
fi

rule "4. is there anything to draw?"
got /agri/stations 8 && green "the catalogue markers are published" \
                     || bad "/agri/stations delivered nothing in 8 s"
got /scan 8 && green "the lidar is publishing" \
            || bad "/scan delivered nothing in 8 s (only the lidar layer is lost)"

rule "verdict"
info "Everything RViz needs is on the wire. If the window is still empty,"
info "the fault is in the view rather than the data:"
info "  - Displays panel, left: any line with a red icon names its own cause"
info "  - Global Options > Fixed Frame must read  map"
info "  - the view is top-down orthographic; scroll to zoom out if the"
info "    camera was left somewhere else by a previous session"
