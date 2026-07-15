# Architecture

## Graphe des nœuds et topics

```
                         ┌──────────────────────────┐
        Webots  ────────▶│  youbot_webots           │
     (smart_agri.wbt)    │  (webots_ros2 driver)    │
                         └───┬───────────┬────────┬──┘
                    /scan    │   /odom   │        │  /camera/image_raw
              (LaserScan)    │ (Odometry)│        │  (Image)
                             ▼           ▼        ▼
        ┌────────────────────────┐   (pose partagée par tous les nœuds)
        │  mapping_node          │◀── /scan, /odom
        │  → /map (OccupancyGrid)│
        └───────────┬────────────┘
                    │ /map
                    ▼
        ┌────────────────────────┐   /goal_pose (PoseStamped)
        │  planning_node         │◀── /map, /odom, /goal_pose
        │  → /plan (Path)        │
        └───────────┬────────────┘
                    │ /plan
                    ▼
        ┌────────────────────────┐
        │  navigation_node       │◀── /plan, /odom
        │  → /cmd_vel (Twist)    │────────────────┐
        └────────────────────────┘                │ /cmd_vel
                                                   ▼
                                         (retour au driver Webots
                                          → 4 vitesses de roues mécanum)

        ┌────────────────────────┐
        │  mission_node          │──▶ /goal_pose   (envoie les buts
        │  (machine à états)     │◀── /odom         un par un)
        └────────────────────────┘
```

## Contrats des topics

| Topic | Type | Publié par | Lu par |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | driver Webots | mapping |
| `/odom` | `nav_msgs/Odometry` | driver Webots | mapping, planning, navigation, mission |
| `/map` | `nav_msgs/OccupancyGrid` | mapping | planning |
| `/goal_pose` | `geometry_msgs/PoseStamped` | mission (ou RViz) | planning |
| `/plan` | `nav_msgs/Path` | planning | navigation |
| `/cmd_vel` | `geometry_msgs/Twist` | navigation | driver Webots |
| `/camera/image_raw` | `sensor_msgs/Image` | driver Webots | (futur nœud vision) |

Le repère : monde XY, origine au centre de l'arène, +X est, +Y nord (REP-103).
`Twist` en frame *corps* : `linear.x` = avance, `linear.y` = gauche,
`angular.z` = lacet CCW.

## Pourquoi une couche `lib/` sans ROS

Les algorithmes (A*, pure pursuit, cinématique mécanum, grille d'occupation,
scan-matching) sont **portés tels quels** depuis le contrôleur Webots validé
et ne dépendent ni de ROS ni de Webots. Bénéfices :

- **Testables seuls** : `pytest` sur un laptop, sans simulateur (voir
  `youbot_control/test/`).
- **Réutilisables des deux côtés** : le driver Webots et le
  `navigation_node` appellent la *même* `body_to_wheel_speeds`, donc le
  comportement reste identique à la Phase A.
- **Un seul endroit à corriger** si un algo évolue.

Les *nœuds* ne contiennent que la plomberie ROS (souscrire, convertir un
message en arguments, appeler l'algo, republier).

## Détail : le scan-matching (Phase 3)

`lib/scan_matcher.py` recale une pose d'odométrie dérivante sur une carte
connue. Deux points clés, validés dans Webots :

1. Le champ de vraisemblance est calculé depuis les **cellules de contour**
   des obstacles (surfaces visibles au lidar), pas les footprints pleins —
   sinon un scan décalé à l'intérieur d'un gros obstacle marque un score
   parfait et l'estimation part à la dérive.
2. Un **garde-fou de confiance** : la recherche coarse+fine ne renvoie
   jamais une pose au score inférieur au prior, ce qui empêche l'emballement
   une fois la correction rebouclée dans l'odométrie.

## Ce qui reste à câbler (TODO)

- **Manipulation** : un serveur d'action `pick` / `unload` (le bras 5-DOF).
  `mission_node` a les marqueurs `TODO` aux bons endroits.
- **Vision** : porter `vision.py` en un nœud qui lit `/camera/image_raw` et
  publie les positions de caisses détectées → alimente `mission_node`.
- **Localisation** : un `localization_node` qui combine odométrie de roues
  (`mecanum.wheel_speeds_to_body`) + `ScanMatcher` pour publier un `/odom`
  corrigé, à la place de la pose GPS parfaite.
- **Orchestration** : remplacer la machine à états par `py_trees_ros` pour
  retrouver l'arbre de comportement de la Phase A.
