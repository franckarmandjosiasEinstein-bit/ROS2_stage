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
