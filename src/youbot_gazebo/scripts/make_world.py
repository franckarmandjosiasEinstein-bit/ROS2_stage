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

import os

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
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {sz/2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material>
            <ambient>{r} {g} {b} 1</ambient>
            <diffuse>{r} {g} {b} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def gutter(name, y):
    # Table-like green culture gutter.
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>0 {y} {GUTTER_H/2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{GUTTER_LEN} {GUTTER_W} {GUTTER_H}</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>{GUTTER_LEN} {GUTTER_W} {GUTTER_H}</size></box></geometry>
          <material>
            <ambient>0.30 0.22 0.12 1</ambient>
            <diffuse>0.40 0.28 0.15 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def plant(name, x, y):
    """A realistic-looking strawberry plant sitting on the gutter top: a brown
    pot, a bushy cluster of green leaves, and several ripe red strawberries.
    Purely visual (well above the 0.20 m lidar plane, so it never clutters the
    map) -- these fruited plants are the harvest targets, not crates on the path."""
    z0 = GUTTER_H
    parts = []
    # Brown pot / root ball on the gutter.
    parts.append(f"""
        <visual name="pot">
          <pose>0 0 {z0 + 0.045:.3f} 0 0 0</pose>
          <geometry><cylinder><radius>0.085</radius><length>0.09</length></cylinder></geometry>
          <material><ambient>0.30 0.18 0.09 1</ambient><diffuse>0.42 0.24 0.12 1</diffuse></material>
        </visual>""")
    # Bushy foliage: several overlapping green spheres.
    for i, (fx, fy, fz, fr) in enumerate(((0.0, 0.0, 0.17, 0.13),
                                          (0.09, 0.05, 0.13, 0.09),
                                          (-0.08, -0.06, 0.14, 0.09),
                                          (0.03, -0.09, 0.12, 0.08),
                                          (-0.05, 0.08, 0.12, 0.08))):
        parts.append(f"""
        <visual name="leaf{i}">
          <pose>{fx} {fy} {z0 + fz:.3f} 0 0 0</pose>
          <geometry><sphere><radius>{fr}</radius></sphere></geometry>
          <material><ambient>0.07 0.32 0.09 1</ambient><diffuse>0.12 0.55 0.15 1</diffuse></material>
        </visual>""")
    # Ripe strawberries hanging around the bush.
    for i, (bx, by, bz) in enumerate(((0.13, 0.02, 0.09), (-0.11, 0.07, 0.08),
                                      (0.05, -0.12, 0.07), (-0.06, -0.10, 0.10),
                                      (0.11, -0.06, 0.11), (-0.12, -0.02, 0.12))):
        parts.append(f"""
        <visual name="berry{i}">
          <pose>{bx} {by} {z0 + bz:.3f} 0 0 0</pose>
          <geometry><sphere><radius>0.032</radius></sphere></geometry>
          <material><ambient>0.6 0.02 0.02 1</ambient><diffuse>0.95 0.10 0.10 1</diffuse></material>
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

    # Perimeter walls (4 boxes just outside +/- arena half extents).
    parts.append(wall("wall_x_pos", ARENA_X_HALF, 0.0, WALL_T, 2 * ARENA_Y_HALF, WALL_H, 0.85, 0.87, 0.90))
    parts.append(wall("wall_x_neg", -ARENA_X_HALF, 0.0, WALL_T, 2 * ARENA_Y_HALF, WALL_H, 0.85, 0.87, 0.90))
    parts.append(wall("wall_y_pos", 0.0, ARENA_Y_HALF, 2 * ARENA_X_HALF, WALL_T, WALL_H, 0.85, 0.87, 0.90))
    parts.append(wall("wall_y_neg", 0.0, -ARENA_Y_HALF, 2 * ARENA_X_HALF, WALL_T, WALL_H, 0.85, 0.87, 0.90))

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

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.75 0.85 0.95 1</background>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision>
        <visual name="v">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><ambient>0.35 0.30 0.25 1</ambient><diffuse>0.45 0.38 0.30 1</diffuse></material>
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
