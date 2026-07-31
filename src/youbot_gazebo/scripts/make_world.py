#!/usr/bin/env python3
"""Generate worlds/greenhouse.sdf for Gazebo Harmonic.

The geometry mirrors the headless sim (sim_node.py) so behaviour is identical,
but here it is a real 3D world the gpu_lidar ray-casts against and RViz/Gazebo
render with colour, lighting and textures:

    Footprint  10 m (X) x 5 m (Y), origin at centre, walls 2.5 m high.
    Gutters    3 x (8.5 x 0.4 x 0.8) at Y = -1.2, 0, +1.2, spanning X[-4.25,4.25].
    Aisles     0.8 m between gutters (Y = +/-0.6); open cross-corridors at X ends.
    Plants     dense rows of strawberry plants on each gutter top: brown pot,
               bushy green foliage and ripe red strawberries (the harvest
               targets). Nothing is placed in the driving aisles.

Run:  python3 scripts/make_world.py   (writes worlds/greenhouse.sdf)
"""

from __future__ import annotations

import math
import os
import random

ARENA_X_HALF = 5.0
ARENA_Y_HALF = 2.5
WALL_H = 2.5
WALL_T = 0.10

GUTTERS_Y = (-1.2, 0.0, 1.2)
GUTTER_LEN = 8.5
GUTTER_W = 0.4
GUTTER_H = 0.8

# Strawberry plants: rows along each gutter top (z = GUTTER_H). No crates on the
# driving path -- the fruited plants themselves are the harvest targets.
PLANT_SPACING = 0.9  # m between plants along a gutter


def wall(name, x, y, sx, sy, sz, r, g, b):
    """Greenhouse glass wall: same collision box as before (lidar/map identical),
    but rendered as a transparent pale-blue pane so the world reads as a real
    glasshouse instead of a grey shed."""
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {sz/2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <transparency>0.62</transparency>
          <material>
            <ambient>0.62 0.74 0.80 1</ambient>
            <diffuse>0.68 0.80 0.88 1</diffuse>
            <specular>0.9 0.9 0.9 1</specular>
          </material>
        </visual>
      </link>
    </model>"""


def post(name, x, y, h=WALL_H, r=0.05):
    """White structural post of the greenhouse frame (visual only)."""
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {h/2:.3f} 0 0 0</pose>
      <link name="link">
        <visual name="v">
          <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
          <material><ambient>0.85 0.86 0.88 1</ambient><diffuse>0.92 0.93 0.95 1</diffuse></material>
        </visual>
      </link>
    </model>"""


def roof(name):
    """Transparent glass roof panel resting on the walls (visual only -- no
    collision, so sensors and physics are untouched)."""
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>0 0 {WALL_H + 0.03:.3f} 0 0 0</pose>
      <link name="link">
        <visual name="v">
          <geometry><box><size>{2*ARENA_X_HALF + 0.4} {2*ARENA_Y_HALF + 0.4} 0.05</size></box></geometry>
          <transparency>0.72</transparency>
          <material>
            <ambient>0.70 0.80 0.86 1</ambient>
            <diffuse>0.75 0.85 0.92 1</diffuse>
            <specular>1 1 1 1</specular>
          </material>
        </visual>
      </link>
    </model>"""


def aisle(name, y):
    """Light gravel strip in the driving aisle (visual only, 1 mm thick)."""
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>0 {y} 0.001 0 0 0</pose>
      <link name="link">
        <visual name="v">
          <geometry><box><size>{2*ARENA_X_HALF - 0.3:.2f} 0.72 0.002</size></box></geometry>
          <material><ambient>0.55 0.53 0.48 1</ambient><diffuse>0.68 0.66 0.60 1</diffuse></material>
        </visual>
      </link>
    </model>"""


def gutter(name, y):
    """Hydroponic culture gutter: white plastic trough (the industry standard
    for strawberries) with a dark soil strip on top. Collision box unchanged --
    the lidar map and planner see exactly the same world as before."""
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>0 {y} 0 0 0 0</pose>
      <link name="link">
        <pose>0 0 {GUTTER_H/2:.3f} 0 0 0</pose>
        <collision name="c"><geometry><box><size>{GUTTER_LEN} {GUTTER_W} {GUTTER_H}</size></box></geometry></collision>
        <visual name="body">
          <geometry><box><size>{GUTTER_LEN} {GUTTER_W} {GUTTER_H}</size></box></geometry>
          <material>
            <ambient>0.80 0.81 0.83 1</ambient>
            <diffuse>0.92 0.93 0.95 1</diffuse>
            <specular>0.3 0.3 0.3 1</specular>
          </material>
        </visual>
        <visual name="soil">
          <pose>0 0 {GUTTER_H/2 + 0.011:.3f} 0 0 0</pose>
          <geometry><box><size>{GUTTER_LEN - 0.06:.2f} {GUTTER_W - 0.08:.2f} 0.025</size></box></geometry>
          <material><ambient>0.20 0.13 0.08 1</ambient><diffuse>0.28 0.19 0.11 1</diffuse></material>
        </visual>
      </link>
    </model>"""


def plant(name, x, y):
    """A realistic-looking strawberry plant sitting on the gutter top: a bushy
    cluster of green leaves and ripe glossy strawberries, each plant slightly
    different (deterministic per-name randomness, so every rebuild is
    identical). Purely visual (well above the 0.20 m lidar plane, so it never
    clutters the map) -- these fruited plants are the harvest targets."""
    rng = random.Random(name)                  # deterministic per plant
    z0 = GUTTER_H
    s = rng.uniform(0.85, 1.15)                # overall plant scale
    parts = []
    # Bushy foliage: overlapping green spheres, two shades of green.
    n_leaf = rng.randint(5, 7)
    for i in range(n_leaf):
        fx = rng.uniform(-0.10, 0.10)
        fy = rng.uniform(-0.09, 0.09)
        fz = rng.uniform(0.10, 0.18) * s
        fr = rng.uniform(0.07, 0.13) * s
        if i == 0:                             # big central tuft
            fx, fy, fz, fr = 0.0, 0.0, 0.17 * s, 0.13 * s
        g = rng.uniform(0.45, 0.62)            # leaf green variation
        parts.append(f"""
        <visual name="leaf{i}">
          <pose>{fx:.3f} {fy:.3f} {z0 + fz:.3f} 0 0 0</pose>
          <geometry><sphere><radius>{fr:.3f}</radius></sphere></geometry>
          <material><ambient>0.06 {g*0.55:.2f} 0.08 1</ambient><diffuse>0.10 {g:.2f} 0.13 1</diffuse></material>
        </visual>""")
    # Ripe strawberries hanging around the bush: glossy red, green calyx cap.
    n_berry = rng.randint(4, 6)
    for i in range(n_berry):
        ang = rng.uniform(0, 6.283)
        rad = rng.uniform(0.10, 0.14)
        bx = rad * math.cos(ang)
        by = rad * 0.8 * math.sin(ang)
        bz = rng.uniform(0.06, 0.12)
        br = rng.uniform(0.028, 0.036)
        red = rng.uniform(0.85, 1.0)           # ripeness shade
        parts.append(f"""
        <visual name="berry{i}">
          <pose>{bx:.3f} {by:.3f} {z0 + bz:.3f} 0 0 0</pose>
          <geometry><sphere><radius>{br:.3f}</radius></sphere></geometry>
          <material>
            <ambient>{red*0.62:.2f} 0.02 0.03 1</ambient>
            <diffuse>{red:.2f} 0.09 0.10 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
        <visual name="calyx{i}">
          <pose>{bx:.3f} {by:.3f} {z0 + bz + br*0.85:.3f} 0 0 0</pose>
          <geometry><sphere><radius>{br*0.45:.3f}</radius></sphere></geometry>
          <material><ambient>0.08 0.30 0.08 1</ambient><diffuse>0.14 0.50 0.14 1</diffuse></material>
        </visual>""")
    visuals = "".join(parts)
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} 0 0 0 0</pose>
      <link name="link">{visuals}
      </link>
    </model>"""


def build() -> str:
    parts = []

    # Perimeter glass walls (4 boxes just outside +/- arena half extents).
    parts.append(wall("wall_x_pos", ARENA_X_HALF, 0.0, WALL_T, 2 * ARENA_Y_HALF, WALL_H, 0.85, 0.87, 0.90))
    parts.append(wall("wall_x_neg", -ARENA_X_HALF, 0.0, WALL_T, 2 * ARENA_Y_HALF, WALL_H, 0.85, 0.87, 0.90))
    parts.append(wall("wall_y_pos", 0.0, ARENA_Y_HALF, 2 * ARENA_X_HALF, WALL_T, WALL_H, 0.85, 0.87, 0.90))
    parts.append(wall("wall_y_neg", 0.0, -ARENA_Y_HALF, 2 * ARENA_X_HALF, WALL_T, WALL_H, 0.85, 0.87, 0.90))

    # White frame posts (corners + mid-span) and the glass roof.
    xs = (-ARENA_X_HALF, -ARENA_X_HALF / 2, 0.0, ARENA_X_HALF / 2, ARENA_X_HALF)
    for i, px in enumerate(xs):
        parts.append(post(f"post_n{i}", px, ARENA_Y_HALF))
        parts.append(post(f"post_s{i}", px, -ARENA_Y_HALF))
    parts.append(roof("roof"))

    # Gravel strips in the two driving aisles (visual only).
    parts.append(aisle("aisle_pos", 0.6))
    parts.append(aisle("aisle_neg", -0.6))

    # Gutters + plants.
    n_plants = int(GUTTER_LEN // PLANT_SPACING)
    for gi, gy in enumerate(GUTTERS_Y):
        parts.append(gutter(f"gutter_{gi}", gy))
        x0 = -(n_plants - 1) * PLANT_SPACING / 2.0
        for pi in range(n_plants):
            parts.append(plant(f"plant_{gi}_{pi}", x0 + pi * PLANT_SPACING, gy))

    models = "".join(parts)
    return f"""<?xml version="1.0" ?>
<!-- Generated by scripts/make_world.py (do not edit by hand).
     Strawberry greenhouse digital twin (10 x 5 m). Geometry matches sim_node.py. -->
<sdf version="1.9">
  <world name="greenhouse">
    <physics name="1ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <!-- GUI camera opens looking straight at the robot spawn (-4.6, 1.9),
         not the world origin, so the YouBot is on screen from frame one. -->
    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </gz-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.6 0.6 0.6</ambient_light>
        <background_color>0.75 0.85 0.95</background_color>
        <camera_pose>-1.8 -1.4 2.6 0 0.55 2.30</camera_pose>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager"/>
      <plugin filename="InteractiveViewControl" name="Interactive view control"/>
      <plugin filename="CameraTracking" name="Camera Tracking"/>
      <plugin filename="WorldControl" name="World control">
        <gz-gui>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">72</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="left" target="left"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </gz-gui>
        <play_pause>true</play_pause>
        <step>true</step>
        <start_paused>false</start_paused>
      </plugin>
      <plugin filename="WorldStats" name="World stats">
        <gz-gui>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">110</property>
          <property type="double" key="width">290</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </gz-gui>
        <sim_time>true</sim_time>
        <real_time>true</real_time>
        <real_time_factor>true</real_time_factor>
      </plugin>
      <plugin filename="EntityTree" name="Entity tree"/>
    </gui>

    <scene>
      <ambient>0.55 0.55 0.52 1</ambient>
      <background>0.72 0.84 0.95 1</background>
      <grid>false</grid>
      <sky></sky>
    </scene>

    <!-- Warm late-morning sun + a soft cool fill light so the glasshouse
         interior is never flat black on the shadow side. -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1.0 0.96 0.86 1</diffuse>
      <specular>0.4 0.38 0.3 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>
    <light type="directional" name="fill">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.28 0.30 0.34 1</diffuse>
      <specular>0 0 0 1</specular>
      <direction>0.4 -0.3 -0.8</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision>
        <visual name="v">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><ambient>0.30 0.24 0.18 1</ambient><diffuse>0.42 0.33 0.24 1</diffuse></material>
        </visual>
      </link>
    </model>
{models}
  </world>
</sdf>
"""


def main() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, "..", "worlds", "greenhouse.sdf")
    with open(os.path.normpath(out), "w") as f:
        f.write(build())
    print(f"wrote {os.path.normpath(out)}")


if __name__ == "__main__":
    main()
