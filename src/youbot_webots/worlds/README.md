# Worlds

Copy your validated Webots world here so the launch file can find it:

```bash
cp ~/Projet/worlds/smart_agriculture.wbt   src/youbot_webots/worlds/
# (optional) the procedural greenhouse too:
cp ~/Projet/worlds/greenhouse.wbt          src/youbot_webots/worlds/
```

Two small edits to the `.wbt` for ROS 2:

1. Set the YouBot `controller` field to **`"<extern>"`** (instead of
   `"my_First_controller"`). `<extern>` tells Webots to let an outside
   process — the `webots_ros2` driver — control the robot.
2. Make sure the robot node has `name "youbot"` so it matches
   `robot_name="youbot"` in `webots.launch.py`.

Nothing else about the world changes: the same bassins, crates, depot,
lidar, camera, GPS and Compass are reused as-is.
