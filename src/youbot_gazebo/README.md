# youbot_gazebo — Phase 3 : le jumeau numérique dans Gazebo

Rendu 3D réaliste (couleur, lumière, ombres, textures) de la serre de fraises,
piloté par **exactement la même pile de contrôle ROS 2** que le mode headless.
Gazebo remplace `sim_node` : le lidar `gpu_lidar` *raytrace la vraie géométrie*,
`VelocityControl` applique `/cmd_vel` (le mécanum *strafe* réellement), et
`OdometryPublisher` fournit une odométrie vérité-terrain dans le repère `map`.

## Contenu

| Fichier | Rôle |
|---|---|
| `worlds/greenhouse.sdf` | Serre 10×5 m : 4 murs, 3 gouttières 8.5×0.4×0.8, fraisiers, 3 caisses rouges. Généré par `scripts/make_world.py`. |
| `urdf/youbot_gz.urdf` | YouBot + `gpu_lidar` (/scan) + plugins `VelocityControl` (/cmd_vel) et `OdometryPublisher` (/odom, TF `map→base_link`). |
| `config/gz_bridge.yaml` | Pont `ros_gz_bridge` : `/clock /cmd_vel /odom /scan /tf`. |
| `launch/gazebo.launch.py` | Démarre Gazebo + spawn robot + pont + `robot_state_publisher` + pile de contrôle + RViz. |

La géométrie est **identique** à `sim_node.py`, donc la navigation validée en
headless se comporte pareil — mais ici en 3D réaliste.

## Pré-requis (ROS 2 Jazzy → Gazebo Harmonic)

```bash
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
                 ros-jazzy-robot-state-publisher ros-jazzy-rviz2
```

## Construire et lancer

```bash
cd ROS2_stage
colcon build --base-paths src
source install/setup.bash

# GUI Gazebo + RViz (par défaut)
ros2 launch youbot_gazebo gazebo.launch.py

# Serveur Gazebo sans fenêtre (juste RViz)
ros2 launch youbot_gazebo gazebo.launch.py gui:=false

# Régénérer le monde après modif des dimensions :
python3 src/youbot_gazebo/scripts/make_world.py
```

Le robot patrouille les allées, découvre les caisses (perception), puis va les
« cueillir » — la même mission qu'en headless, dans un décor 3D réaliste.

## The drivetrain model (`drive_model_node`)

Gazebo's `VelocityControl` is kinematic: command a body twist, get it,
instantly and exactly. Fine for building a mission layer, useless for two
things this project now needs — honest sim-to-real numbers, and a plant on
which predictive control can demonstrate any advantage at all.

`drive_model_node` sits between `/cmd_vel` and the bridge:

```
planners -> /cmd_vel_raw -> safety_node -> /cmd_vel
                                              |
                                      drive_model_node       <- the PLANT
                                              |
                                        /cmd_vel_exec -> bridge -> Gazebo
```

It models motors, never policy. `safety_node` remains the single writer of
`/cmd_vel`; nothing here decides where the robot goes.

| Effect | Default | Why it matters |
|---|---|---|
| Command delay | 60 ms | the `t_react` of ISO 13855; a protective distance is not geometry alone |
| First-order lag | τ = 80 ms | at 20 Hz that is a whole control period of phase |
| Wheel acceleration limit | 25 rad/s² | finite torque; commanding a step is what breaks roller traction |
| Wheel speed limit | 14 rad/s | applied **per wheel** |
| Slip | 1.5 / 6.0 / 9.0 % | longitudinal / lateral / rotational — rollers do not slip equally in all directions |

### The one thing that cannot be faked by adding noise

Limits are applied **in wheel space**: the body twist is resolved through the
mecanum inverse kinematics, each wheel is lagged, rate-limited and saturated
on its own, then mapped back through the forward kinematics. Saturating a body
twist scales it and keeps its direction. Saturating one wheel of a diagonal
pair **changes the direction**:

```
commanded (vx, vy, wz) = (0.90, 0.00, 1.40)
achieved               = (0.53, 0.00, 0.44)
vx keeps 59 % of its command, wz only 32 %
```

The robot curves differently from what was asked. That coupling is what a real
mecanum base does, it is invisible to a kinematic plant, and it is precisely
the class of problem constrained MPC exists to handle. `check_regressions.py`
asserts it numerically so a "simplifying" rewrite cannot quietly remove it.

### Slip is published as two topics, not one

`/wheel_speeds` is what the encoders would report; `/cmd_vel_exec` is what the
body actually does. Their difference **is** odometry drift — not a random walk
added to a pose afterwards, but the physical mechanism. This is what makes the
stage-2 UMBmark calibration meaningful in simulation rather than a formality.

### Running it

Off by default, so the measured baseline in the report stays reproducible and
a mission-layer regression can never be blamed on the plant changing:

```bash
ros2 launch youbot_gazebo gazebo.launch.py                    # kinematic (baseline)
ros2 launch youbot_gazebo gazebo.launch.py drive_model:=true  # with a drivetrain
```

Run both and compare `perf_monitor`. The difference is the cost of reality,
and it is the number that says whether the current controller is ready for
hardware.
