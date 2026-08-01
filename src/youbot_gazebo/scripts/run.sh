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
  pkill -9 -f "gui_follow"        2>/dev/null
  # Say so out loud instead of leaving it to be guessed from the next run
  # misbehaving. SIGKILL cannot be refused, so anything still listed here is a
  # process whose command line none of the patterns above match -- printed with
  # its name, so the missing pattern can just be added.
  sleep 1
  local left
  left=$(pgrep -a -f "gz sim|gz-sim|parameter_bridge|rviz2|rqt_image_view|youbot_" \
         2>/dev/null | grep -v "run.sh" | grep -v pgrep)
  if [ -n "$left" ]; then
    echo "[run.sh] STILL ALIVE after cleanup -- report these:"
    printf '%s\n' "$left" | sed 's/^/[run.sh]   /'
  else
    echo "[run.sh] no sim process left alive (checked)."
  fi
  echo "[run.sh] done."
}
trap cleanup EXIT INT TERM

# Gazebo rewrites this on every exit with the camera pose you left behind, and
# reads it back in preference to the world's <gui> pose -- so a run started
# after you parked the camera on empty space opens blind. Drop it; the world's
# pose applies, and the launch also locks the camera onto the robot.
rm -f "$HOME/.gz/sim/8/gui.config" 2>/dev/null

# Never start on top of a ghost sim from a previous crash/Ctrl-C.
if pgrep -f "gz sim" >/dev/null 2>&1; then
  echo "[run.sh] leftover gz sim found -- cleaning up first."
  cleanup
  sleep 1
fi

# Entry-point metadata check. With --symlink-install the console scripts under
# install/ resolve their entry points through an egg-link back to
# src/*.egg-info, and when that metadata goes missing EVERY node dies at import
# with "PackageNotFoundError: No package metadata was found for youbot-control"
# -- fifteen identical tracebacks that say nothing about the actual fix. Catch
# it here instead, in one line.
if ! python3 -c "
import importlib.metadata as m
for d in ('youbot-control', 'youbot-slam'):
    m.distribution(d)
" 2>/dev/null; then
  echo "[run.sh] The installed packages carry no entry-point metadata, so every"
  echo "[run.sh] node would die on startup. Rebuild from clean:"
  echo
  echo "    rm -rf build install log && colcon build --symlink-install"
  echo "    source install/setup.bash"
  echo
  echo "[run.sh] If it comes back, drop --symlink-install: a plain colcon build"
  echo "[run.sh] writes real dist metadata and does not depend on src/ at all."
  exit 1
fi

if [ "$1" = "slam" ]; then
  shift
  ros2 launch youbot_slam gazebo_slam.launch.py "$@"
else
  ros2 launch youbot_gazebo gazebo.launch.py "$@"
fi
