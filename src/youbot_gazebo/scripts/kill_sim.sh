#!/usr/bin/env bash
# Emergency broom: kill EVERYTHING related to the simulation, whatever way it
# was launched. Use when a run was interrupted and the next one misbehaves
# (ghost world, robot invisible in the GUI, two /clock publishers...).
#
#   ./src/youbot_gazebo/scripts/kill_sim.sh
for pat in "gz sim" "gz-sim" "ruby.*gz sim" "parameter_bridge" \
           "robot_state_publisher" "rviz2" "rqt_image_view" \
           "youbot_control" "youbot_slam" "slam_toolbox" "ros2 launch"; do
  pkill -9 -f "$pat" 2>/dev/null
done
sleep 0.5
left=$(pgrep -af "gz sim|gz-sim|parameter_bridge|rviz2" | wc -l)
echo "[kill_sim] done ($left related process(es) left)."
