# YouBot — Autonomous Driving for Smart Agriculture (ROS 2)

Portage vers **ROS 2** du système de commande autonome développé et validé
dans Webots (KUKA YouBot, base mécanum, bras 5-DOF, scénario d'agriculture
intelligente). Objectif : passer d'un contrôleur Webots monolithique à une
architecture ROS 2 distribuée, propre et modulaire.

> Nouveau sur ROS 2 ? Lis **[`docs/ROS2_PRIMER.md`](docs/ROS2_PRIMER.md)**
> d'abord (15 min), puis **[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)**
> pour le graphe des nœuds.

## Ce qu'il y a dans la boîte

| Package | Rôle |
|---|---|
| **`youbot_control`** | Le cerveau : nœuds `mapping`, `planning`, `navigation`, `mission` + la couche `lib/` (algorithmes portés, sans ROS, testés) |
| **`youbot_bringup`** | Fichiers de lancement + paramètres pour démarrer toute la pile |
| **`youbot_webots`** | Intégration `webots_ros2` : plugin driver mécanum, URDF des capteurs, launch du simulateur |

Chaque brique de la Phase A (Webots) a son équivalent ici — voir le tableau
de correspondance dans le PRIMER.

## Prérequis

1. **Ubuntu + ROS 2** (Humble sur 22.04, ou Jazzy sur 24.04 — les deux
   marchent). Vérifie ce que tu as :
   ```bash
   printenv ROS_DISTRO      # -> humble, jazzy, ...
   ```
   Si vide : `source /opt/ros/<distro>/setup.bash` (ajoute-le à ton
   `~/.bashrc`).

2. **Dépendances** (remplace `$ROS_DISTRO` par ta distro) :
   ```bash
   sudo apt update
   sudo apt install ros-$ROS_DISTRO-webots-ros2 python3-colcon-common-extensions python3-numpy
   ```
   `webots-ros2` installe le pont officiel Cyberbotics (et peut installer
   Webots si besoin).

## Installation

```bash
git clone https://github.com/franckarmandjosiasEinstein-bit/youbot_ros2_smart_agriculture.git
cd youbot_ros2_smart_agriculture

# récupère les dépendances déclarées dans les package.xml
rosdep install --from-paths src --ignore-src -r -y   # optionnel mais recommandé

colcon build --symlink-install
source install/setup.bash
```

## Lancer

### Option A — la pile de contrôle seule (sans simulateur)
Utile pour vérifier que les nœuds démarrent et se parlent. Ils attendront
des `/scan` et `/odom` (que tu peux fournir via un bag ou un faux publisher).
```bash
ros2 launch youbot_bringup bringup.launch.py
# dans un autre terminal :
ros2 node list        # -> /mapping_node /planning_node /navigation_node /mission_node
ros2 topic list
rqt_graph
```

### Option B — avec Webots (boucle complète)
```bash
# 1) copie ton monde (voir src/youbot_webots/worlds/README.md) et passe
#    le controller du YouBot à "<extern>", name "youbot".
# 2) démarre le simulateur + le driver :
ros2 launch youbot_webots webots.launch.py
# 3) démarre le cerveau :
ros2 launch youbot_bringup bringup.launch.py
```
Envoie un but à la main pour tester la planif+nav :
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 3.6, y: -1.57}, orientation: {w: 1.0}}}'
```

## Tester les algorithmes (sans ROS ni Webots)

Les algos vivent dans `youbot_control/lib/` et sont testables seuls :
```bash
colcon test --packages-select youbot_control
colcon test-result --verbose
# ou directement :
python3 -m pytest src/youbot_control/test -v
```

## Dépannage

**`PackageNotFoundError: No package metadata was found for youbot-control`**
— tous les nœuds meurent au démarrage, Gazebo tourne quand même.

`colcon build --symlink-install` ne copie pas le code : il pose un lien vers
`src/`, et les scripts installés retrouvent leur `main()` via les métadonnées
`src/<paquet>.egg-info`. Quand ce dossier manque ou vient d'un autre
setuptools, tous les nœuds Python échouent à l'import avec la même trace.
Reconstruire à neuf :

```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

Si le problème revient, enlever `--symlink-install` : un `colcon build` normal
copie le code et écrit de vraies métadonnées, sans dépendre de `src/`. `run.sh`
vérifie ce point avant de lancer quoi que ce soit et affiche la commande.

## État d'avancement

- [x] Squelette du workspace + 3 packages qui compilent
- [x] Algorithmes portés depuis Webots (`lib/`) + tests unitaires verts
- [x] Nœuds mapping / planning / navigation reliés par topics
- [x] Driver `webots_ros2` (mécanum `/cmd_vel`→roues, GPS+Compass→`/odom`)
- [x] Orchestrateur de mission (machine à états)
- [ ] Nœud vision (port de `vision.py`, lit `/camera/image_raw`)
- [ ] Serveur d'action manipulation (bras 5-DOF : pick / unload)
- [ ] `localization_node` (odométrie roues + scan-matching → `/odom` corrigé)
- [ ] Orchestration via `py_trees_ros` (arbre de comportement complet)

Détail des TODO dans [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md).

## Licence
MIT.
