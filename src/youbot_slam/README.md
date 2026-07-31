# youbot_slam — autonomie niveau 3 (localisation autonome)

Jusqu'ici la pile de contrôle recevait sa position de l'extérieur (odométrie
vérité-terrain de Gazebo). Ce package retire cette connaissance a priori : le
robot se localise **dans la carte qu'il construit lui-même**. C'est la brique
qui rend la plateforme générique — serre, hôpital, entrepôt : même logiciel.

## Nœuds

| Nœud | Rôle |
|---|---|
| `noisy_odom` | Dégrade l'odométrie parfaite de Gazebo en odométrie réaliste qui dérive (biais d'échelle 4 %/8 %/5 % + bruit blanc, modèle encodeurs mécanum). Sans elle, un SLAM n'aurait rien à prouver en simulation. |
| `slam_node` | SLAM incrémental fait maison : prédiction (odométrie) → correction (scan matching corrélatif sur champ de vraisemblance) → mise à jour de la carte log-odds, **si** le matching est confiant. Publie `/pose_slam` + TF `map→base_link` et logge l'erreur vs vérité terrain. |
| `pose_from_tf` | Adaptateur pour slam_toolbox : recompose TF `map→base_link` en Odometry sur `/pose_slam`. |

## Les trois pièges résolus (voir `matcher.py`)

1. **Appariement des angles** — le matcher hérité de Webots suppose un balayage
   horaire ; un LaserScan ROS balaie en sens inverse. Layout exact injecté.
2. **Verrou de couloir** — avec une carte auto-construite, la zone devant le
   robot est inexplorée : un score nul là-bas tire la pose vers l'arrière (sur
   le banc, l'erreur croissait exactement à la vitesse du robot). Les rayons en
   zone inconnue reçoivent un score neutre 0,5.
3. **Rétroaction carte↔pose** — intégrer un scan à une pose légèrement fausse
   étale les murs dans la carte, que le matcher réapprend ensuite (divergence
   ~0,1 m/scan). La carte n'est mise à jour que si la qualité du matching
   dépasse un seuil.

## Résultats (banc sans ROS : patrouille 18 m, dérive injectée 4-8 %)

| Localisation | Moyenne | Max | Finale |
|---|---|---|---|
| Odométrie seule | 0,51 m | 1,16 m | 1,16 m |
| **SLAM maison** | **0,06 m** | **0,25 m** | **0,05 m** |

## Lancer

```bash
# Niveau 3 complet (Gazebo + SLAM maison), pile de contrôle INCHANGÉE
# (elle est simplement re-câblée sur /pose_slam par remapping) :
ros2 launch youbot_slam gazebo_slam.launch.py

# Démo d'échec (odométrie qui dérive, sans correction) :
ros2 launch youbot_slam gazebo_slam.launch.py slam:=false pose_topic:=odom_noisy

# Comparaison industrielle (slam_toolbox comme backend) :
sudo apt install ros-jazzy-slam-toolbox
ros2 launch youbot_slam gazebo_slam_toolbox.launch.py
```

Le log de `slam_node` affiche en continu la métrique pour le rapport :

    pose error vs truth: SLAM 0.05 m | odometry alone 0.83 m
