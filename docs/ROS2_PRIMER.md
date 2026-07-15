# ROS 2 en 15 minutes (pour ce projet)

Tu viens de Webots où **un seul script** (`my_First_controller.py`) faisait
tout. ROS 2 découpe ce script en plusieurs **programmes indépendants**
(les *nœuds*) qui se parlent par **messages**. C'est plus de fichiers, mais
chaque morceau devient testable et remplaçable seul.

## Les 5 mots à retenir

| Concept | C'est quoi | Dans ce projet |
|---|---|---|
| **Node** (nœud) | un programme qui fait une chose | `mapping_node`, `navigation_node`... |
| **Topic** | un canal nommé où circulent des messages | `/scan`, `/map`, `/cmd_vel` |
| **Message** | la structure de données échangée | `LaserScan`, `OccupancyGrid`, `Twist` |
| **Publisher / Subscriber** | qui écrit / qui lit un topic | mapping *publie* `/map`, planning *lit* `/map` |
| **Package** | un dossier livrable (code + deps) | `youbot_control`, `youbot_webots`... |

Analogie : un topic est une **station de radio**. Un nœud qui *publie*
émet ; un nœud qui *souscrit* écoute. Plusieurs peuvent écouter la même
station, et personne n'a besoin de connaître l'autre — juste le nom de la
station et le type d'émission (le message).

## Le workspace et colcon

```
youbot_ros2_smart_agriculture/     <- le "workspace"
├── src/                           <- TON code (versionné sur git)
│   ├── youbot_control/            <- un package
│   ├── youbot_bringup/
│   └── youbot_webots/
├── build/  install/  log/         <- généré par colcon (jamais versionné)
```

Cycle de travail, à chaque modif :

```bash
cd ~/youbot_ros2_smart_agriculture
colcon build --symlink-install     # compile/installe les packages
source install/setup.bash          # rend tes nœuds visibles au shell
ros2 launch youbot_bringup bringup.launch.py
```

> `--symlink-install` : les fichiers Python sont liés, pas copiés — tu
> édites un `.py` et tu relances sans recompiler. Pratique pour apprendre.

## Commandes de survie

```bash
printenv ROS_DISTRO            # quelle distro ai-je ? (humble / jazzy...)
ros2 node list                # quels nœuds tournent ?
ros2 topic list               # quels topics existent ?
ros2 topic echo /odom         # voir passer les messages en direct
ros2 topic hz /scan           # à quelle fréquence ça publie ?
ros2 topic info /map -v       # qui publie / qui écoute ce topic
ros2 run youbot_control mapping_node   # lancer UN nœud à la main
rqt_graph                     # dessine le graphe nœuds<->topics
```

## Comment le code Webots se transpose

| Webots (Phase A) | ROS 2 (Phase B) |
|---|---|
| `mapping.py` | `youbot_control/lib/occupancy_grid.py` (algo) + `mapping_node.py` (glue) |
| `planning.py` | `lib/astar.py` + `planning_node.py` |
| `navigation.py` | `lib/mecanum.py` + `lib/pure_pursuit.py` + `navigation_node.py` |
| `localization.py` | `lib/scan_matcher.py` (+ un futur `localization_node`) |
| `behavior_trees.py` | `mission_node.py` (machine à états ; plus tard `py_trees_ros`) |
| `getDevice()` / `wb_*` | le driver `webots_ros2` + les topics |

**L'idée clé** : les *algorithmes* validés dans Webots vivent dans `lib/`,
sans aucune dépendance ROS. On peut donc les `pytest` sur un laptop. Les
*nœuds* ne font que traduire des messages ↔ appels de fonctions. Si un
algo marchait dans Webots, il marche pareil ici — seule la plomberie change.
