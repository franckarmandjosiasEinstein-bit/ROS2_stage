#!/usr/bin/env bash
# Convenience wrapper so the launcher is where you expect it: at the top of the
# workspace. Everything lives in src/youbot_gazebo/scripts/run.sh.
#
#   ./run.sh              # level 2 (ground-truth pose)
#   ./run.sh slam         # level 3 (SLAM localisation)
#   ./run.sh slam gui:=false
exec "$(dirname "$0")/src/youbot_gazebo/scripts/run.sh" "$@"
