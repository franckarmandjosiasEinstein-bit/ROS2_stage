#!/usr/bin/env bash
# Launch the Gazebo digital twin and GUARANTEE every child process is killed on
# exit. gz sim does not die on Ctrl-C, and a leftover sim keeps publishing an
# old /clock -> the next run then has two clocks and RViz thrashes. Running the
# stack through this wrapper (with a cleanup trap) avoids that entirely.
#
#   ./src/youbot_gazebo/scripts/run.sh            # GUI + RViz
#   ./src/youbot_gazebo/scripts/run.sh gui:=false  # headless server
set -m

cleanup() {
  echo
  echo "[run.sh] shutting down -- killing all sim processes..."
  pkill -9 -f "gz sim"            2>/dev/null
  pkill -9 -f "gz-sim"            2>/dev/null
  pkill -9 -f "ruby.*gz sim"      2>/dev/null
  pkill -9 -f "parameter_bridge"  2>/dev/null
  pkill -9 -f "robot_state_publisher" 2>/dev/null
  pkill -9 -f "rviz2"             2>/dev/null
  echo "[run.sh] done."
}
trap cleanup EXIT INT TERM

ros2 launch youbot_gazebo gazebo.launch.py "$@"
