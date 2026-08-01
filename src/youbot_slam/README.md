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
   le banc, l'erreur croissait exactement à la vitesse du robot).

   Premier correctif : score neutre 0,5 pour les rayons en zone inconnue. Il
   n'a pas suffi, et la vraie cause n'était pas celle-là. Le score était une
   **somme** sur les rayons, donc une pose gagnait des points simplement en
   plaçant *plus* de rayons sur du terrain déjà cartographié — c'est-à-dire en
   reculant. Mesuré au banc, quatre poses candidates espacées de 0,10 m dans un
   couloir dont la carte s'arrête devant le robot : 56, 54, 52 et 50 rayons sur
   du connu. Reculer de 0,10 m « rapportait » trois rayons appariés, quelle que
   soit la valeur donnée aux cellules inconnues.

   Conséquence sur le terrain (run de 36 min en serre) : l'estimée a terminé
   **1,20 m en arrière** de la vérité *le long du cap*, alors que l'odométrie
   qu'on lui fournissait restait à 0,22 m — le robot roulait vers des dépôts
   qu'il avait déjà dépassés, tournait en rond et n'a récolté que 10 fraises
   au lieu de 39.

   Correctif retenu : le score est la **moyenne** des poids sur les rayons
   tombés en zone cartographiée. Avoir plus ou moins de rayons ne rapporte
   plus rien en soi ; seule la *qualité* de l'appariement vote. Même banc :
   0,988 / 0,991 / 0,994 / 0,993 le long du couloir (plat, comme doit l'être
   un axe non observable) contre 0,994 à la vérité et 0,688 à 0,10 m de
   travers. Un plancher `min_beams` empêche une pose qui ne voit presque rien
   de marquer 1,0 avec trois rayons chanceux.

   **Et ça n'a toujours pas suffi** : 2,03 m de dérive en 13 min, contre
   1,20 m en 36 min avant. La moyenne était plus juste en principe et pire en
   pratique, ce qui a fini par désigner le vrai coupable.

   **La frontière.** Un rayon qui *frôle* une cellule sans s'y arrêter y
   ajoute `L_FREE = -0,40`. L'ancienne porte acceptait toute cellule avec
   |log-odds| > 0,2 : une seule rasance suffisait donc à déclarer une cellule
   « connue et vide ». Or la bande juste devant le robot en est pleine —
   connue, mais pas encore résolue comme obstacle. Un rayon qui y atterrit
   n'était pas compté comme *inexploré*, il était compté comme **raté**.
   Avancer était donc activement puni ; et sous une moyenne chaque raté tire
   la note vers le bas au lieu de simplement ne rien ajouter, d'où
   l'aggravation.

   Banc dédié (couloir cartographié jusqu'à x = 0, cellules frôlées jusqu'à
   x = +0,6), `correct_online` appelé **à la pose vraie** :

   | porte | qualité | correction appliquée |
   |---|---|---|
   | |L| > 0,2 (ancienne) | 0,80 | **−0,110 m** (fenêtre pleine, en arrière) |
   | |L| > 2,0 (`mature_log_odds`) | 0,99 | **0,000 m** |

   Une cellule doit maintenant avoir été vue ~5 fois (5 × 0,40) ou touchée
   3 fois (3 × 0,85) avant d'avoir droit au vote. Le champ de distance, lui,
   reste généreux : un seul impact suffit à semer une surface, car un obstacle
   manquant créerait de faux ratés.

   **Gain anisotrope** (`along_gain_scale`). Le long d'une allée droite, la
   position n'est pas observable — glisser vers l'avant ne change rien à ce
   que voit le lidar. En travers et en cap, les rangs de plants informent
   vraiment. Un EKF obtient ça gratuitement via sa covariance ; avec un filtre
   à gain fixe il faut l'écrire : gain plein en travers et en cap, 15 % le
   long du cap.

   Filet de sécurité ajouté au passage : `slam_node` cumule les corrections
   appliquées, projetées le long du cap et en travers, et **prévient dans le
   log** dès que le scan matching s'est éloigné de plus de `max_drift_warn`
   (0,60 m) de la navigation à l'estime. Un cumul *négatif* le long du cap =
   l'estimée retarde sur le robot. Plus besoin de le reconstituer après coup.
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
