#!/usr/bin/env bash
# Launch the Gazebo digital twin and GUARANTEE every child process is killed on
# exit. gz sim does not die on Ctrl-C, and a leftover sim keeps publishing an
# old /clock -> the next run then has two clocks, RViz thrashes, and the GUI
# can even end up attached to a ghost world without the robot. Running the
# stack through this wrapper (with a cleanup trap) avoids that entirely.
#
#   ./src/youbot_gazebo/scripts/run.sh                 # level 2 (ground truth)
#   ./src/youbot_gazebo/scripts/run.sh slam            # level 3 (homemade SLAM)
#   ./src/youbot_gazebo/scripts/run.sh gui:=false      # headless server
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
  pkill -9 -f "rqt_image_view"    2>/dev/null
  pkill -9 -f "youbot_control"    2>/dev/null
  pkill -9 -f "youbot_slam"       2>/dev/null
  echo "[run.sh] done."
}
trap cleanup EXIT INT TERM

# Never start on top of a ghost sim from a previous crash/Ctrl-C.
if pgrep -f "gz sim" >/dev/null 2>&1; then
  echo "[run.sh] leftover gz sim found -- cleaning up first."
  cleanup
  sleep 1
fi

if [ "$1" = "slam" ]; then
  shift
  ros2 launch youbot_slam gazebo_slam.launch.py "$@"
else
  ros2 launch youbot_gazebo gazebo.launch.py "$@"
fi
