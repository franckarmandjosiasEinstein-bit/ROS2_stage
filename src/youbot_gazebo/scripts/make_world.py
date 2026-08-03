#!/usr/bin/env python3
"""Generate worlds/greenhouse.sdf for Gazebo Harmonic.

The geometry mirrors the headless sim (sim_node.py) so behaviour is identical,
but here it is a real 3D world the gpu_lidar ray-casts against and RViz/Gazebo
render with colour, lighting and textures:

    Footprint  10 m (X) x 5 m (Y), origin at centre, walls 2.5 m high.
    Gutters    3 x (8.0 x 0.4 x 0.8) at Y = -1.2, 0, +1.2, spanning X[-4,4].
    Aisles     0.8 m between gutters (Y = +/-0.6); open cross-corridors at X ends.
    Plants     dense rows of strawberry plants on each gutter top: brown pot,
               bushy green foliage and strawberries at a continuous, randomised
               ripeness (the harvest targets). Nothing is placed in the
               driving aisles.
    Distractors a handful of red, non-fruit objects (crates, pipes, a dropped
               jacket) scattered in the aisles -- hard negatives for the
               colour-ratio failure mode the ml/ pipeline exists to fix.

Domain randomisation (this is what varies a capture SESSION):
    colour temperature, sun direction + intensity  -- one of four lighting
        presets (cold_morning, neutral_midday, warm_late_afternoon,
        overcast), sampled per run unless --condition pins one;
    fruit maturity                                  -- continuous per-berry
        ripeness in [0, 1], green to red, recorded in berries.yaml;
    red distractors                                 -- count and placement
        randomised per run.
Everything else (arena, gutters, aisles) is fixed geometry so the lidar map
and planner are unaffected.

Run:
    python3 scripts/make_world.py --condition warm_late_afternoon --seed 7
    python3 scripts/make_world.py                # random condition, random seed

Writes worlds/greenhouse.sdf, worlds/berries.yaml and worlds/conditions.json.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random

ARENA_X_HALF = 5.0
ARENA_Y_HALF = 2.5
WALL_H = 2.5
WALL_T = 0.10

GUTTERS_Y = (-1.2, 0.0, 1.2)
GUTTER_LEN = 8.0   # 8.5 pinched the end-of-row corridors to ~6 cm once both
                   # the gutter ends and the walls were inflated by 0.32 m --
                   # the robot jammed in the corners. 8.0 leaves ~0.3 m clear.
GUTTER_W = 0.4
GUTTER_H = 0.8

# Strawberry plants: rows along each gutter top (z = GUTTER_H). No crates on the
# driving path -- the fruited plants themselves are the harvest targets.
PLANT_SPACING = 0.9  # m between plants along a gutter

# Lighting presets a capture session is sampled from. Ranges, not points --
# two "warm_late_afternoon" sessions should not render identically, only
# recognisably as the same time of day. Elevation/azimuth are the sun's
# position in the sky (degrees); intensity scales the light's own brightness;
# ambient_scale scales the scene fill so overcast frames are not just a dim
# sun with everything else unlit, which is not what overcast looks like.
CONDITIONS = {
    "cold_morning": dict(
        temp_k=(7500, 9500), elev_deg=(12, 22), azim_deg=(60, 100),
        intensity=(0.55, 0.75), ambient_scale=(0.85, 1.00), overcast=False),
    "neutral_midday": dict(
        temp_k=(5300, 6000), elev_deg=(65, 85), azim_deg=(0, 360),
        intensity=(0.90, 1.05), ambient_scale=(0.95, 1.05), overcast=False),
    "warm_late_afternoon": dict(
        temp_k=(2600, 3400), elev_deg=(8, 18), azim_deg=(240, 290),
        intensity=(0.55, 0.80), ambient_scale=(0.80, 0.95), overcast=False),
    "overcast": dict(
        temp_k=(6500, 8000), elev_deg=(35, 60), azim_deg=(0, 360),
        intensity=(0.22, 0.40), ambient_scale=(1.25, 1.45), overcast=True),
}

# Continuous ripeness endpoints: 0.0 is unripe green, 1.0 is fully ripe red.
# ml/README.md calls out "ripe, half-ripe, green -- ripeness is a continuum,
# all three present" as a coverage requirement, not colour classes.
UNRIPE_RGB = (0.16, 0.42, 0.10)
RIPE_RGB = (0.95, 0.05, 0.06)

# Red, but not fruit: what actually causes the colour-threshold false
# positives per ml/README.md ("pipes, tools, crates, clothing"). Each entry is
# (shape, base_rgb, specular, size_range_m) -- deliberately NOT the berry's
# hue/gloss combination, since the point is a class the ratio test cannot
# tell from a strawberry by colour alone, not a copy of the berry material.
DISTRACTOR_KINDS = (
    ("box", (0.55, 0.10, 0.08), 0.15, (0.10, 0.30)),   # red plastic crate
    ("cylinder", (0.62, 0.18, 0.05), 0.35, (0.03, 0.06)),  # rust-red pipe/tool
    ("box", (0.50, 0.06, 0.12), 0.05, (0.10, 0.22)),   # dropped red jacket/rag
)


def kelvin_to_rgb(temp_k: float) -> tuple[float, float, float]:
    """Blackbody colour for a temperature in Kelvin, as 0..1 RGB.

    Tanner Helland's fit to Mitchell Charity's blackbody table: the standard
    cheap approximation, accurate enough for tinting a directional light and
    far cheaper than a spectral render."""
    t = max(1000.0, min(40000.0, temp_k)) / 100.0
    r = 255.0 if t <= 66 else 329.698727446 * ((t - 60) ** -0.1332047592)
    if t <= 66:
        g = 99.4708025861 * math.log(t) - 161.1195681661
    else:
        g = 288.1221695283 * ((t - 60) ** -0.0755148492)
    if t >= 66:
        b = 255.0
    elif t <= 19:
        b = 0.0
    else:
        b = 138.5177312231 * math.log(t - 10) - 305.0447927307

    def clamp(v):
        return max(0.0, min(255.0, v)) / 255.0

    return clamp(r), clamp(g), clamp(b)


def sun_direction(elev_deg: float, azim_deg: float) -> tuple[float, float, float]:
    """Unit direction the light travels for a sun at (elevation, azimuth).

    elev_deg=90 is straight down (noon); low elev_deg is a low sun and the
    long, near-horizontal shadows of morning/evening."""
    elev, azim = math.radians(elev_deg), math.radians(azim_deg)
    return (-math.cos(elev) * math.cos(azim),
            -math.cos(elev) * math.sin(azim),
            -math.sin(elev))


def sample_domain(rng: random.Random, condition: str) -> dict:
    """Sample one capture session's domain parameters.

    `condition` pins a preset (reproducible together with --seed); "random"
    (the default) samples one of the four uniformly, which is what you want
    across many unattended sessions so the dataset ends up balanced rather
    than however many times a human remembered to pass --condition."""
    if condition == "random":
        condition = rng.choice(sorted(CONDITIONS))
    spec = CONDITIONS[condition]
    return dict(
        condition=condition,
        temp_k=rng.uniform(*spec["temp_k"]),
        elev_deg=rng.uniform(*spec["elev_deg"]),
        azim_deg=rng.uniform(*spec["azim_deg"]),
        intensity=rng.uniform(*spec["intensity"]),
        ambient_scale=rng.uniform(*spec["ambient_scale"]),
        overcast=spec["overcast"],
        n_distractors=rng.randint(4, 10),
    )


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


# True world positions of every berry, filled in by plant() and dumped next to
# the world. This is the ground-truth reference the debug monitor compares the
# robot's perception against -- generated, never hand-maintained.
TRUE_BERRIES = []
# Continuous ripeness in [0, 1], one entry per TRUE_BERRIES entry, same order.
# A separate list (not a 5th tuple element) on purpose: truth_monitor.py and
# berry_view.py unpack berries.yaml's "berries:" rows as fixed (x, y, z, r)
# 4-tuples, so widening that row would break both. A new "ripeness:" section
# is invisible to code that only reads a section it names explicitly.
TRUE_RIPENESS = []
# Every foliage sphere, as an OCCLUDER. A berry being inside the camera frustum
# does not mean the camera can see it: half the fruit on a plant hangs behind
# that plant's own leaves. Without this, "berries truly in view" counts fruit
# no camera could ever report and every run reads as a perception failure.
TRUE_FOLIAGE = []


def plant(name, x, y, seed):
    """A realistic-looking strawberry plant sitting on the gutter top: a bushy
    cluster of green leaves and strawberries at a continuous ripeness, each
    plant slightly different. Seeded from (name, seed) so a given session
    seed reproduces exactly, but different sessions -- and therefore
    different capture datasets -- see different foliage and fruit. Purely
    visual (well above the 0.20 m lidar plane, so it never clutters the map)
    -- these fruited plants are the harvest targets."""
    rng = random.Random(f"{name}:{seed}")
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
        TRUE_FOLIAGE.append((x + fx, y + fy, z0 + fz, fr))
        g = rng.uniform(0.45, 0.62)            # leaf green variation
        parts.append(f"""
        <visual name="leaf{i}">
          <pose>{fx:.3f} {fy:.3f} {z0 + fz:.3f} 0 0 0</pose>
          <geometry><sphere><radius>{fr:.3f}</radius></sphere></geometry>
          <material><ambient>0.06 {g*0.55:.2f} 0.08 1</ambient><diffuse>0.10 {g:.2f} 0.13 1</diffuse></material>
        </visual>""")
    # Strawberries hanging around the bush at a continuous ripeness: glossy,
    # green calyx cap. ripeness=0 is green and unripe, 1 is fully ripe red --
    # sampled per berry, not per plant, so a single bush shows fruit at
    # several stages at once, same as a real one.
    n_berry = rng.randint(4, 6)
    for i in range(n_berry):
        ang = rng.uniform(0, 6.283)
        rad = rng.uniform(0.10, 0.14)
        bx = rad * math.cos(ang)
        by = rad * 0.8 * math.sin(ang)
        bz = rng.uniform(0.06, 0.12)
        br = rng.uniform(0.028, 0.036)
        ripeness = rng.uniform(0.0, 1.0)
        shade = rng.uniform(0.92, 1.05)        # per-berry gloss/shade jitter
        cr = min(1.0, max(0.0, UNRIPE_RGB[0] + (RIPE_RGB[0] - UNRIPE_RGB[0]) * ripeness) * shade)
        cg = min(1.0, max(0.0, UNRIPE_RGB[1] + (RIPE_RGB[1] - UNRIPE_RGB[1]) * ripeness) * shade)
        cb = min(1.0, max(0.0, UNRIPE_RGB[2] + (RIPE_RGB[2] - UNRIPE_RGB[2]) * ripeness) * shade)
        TRUE_BERRIES.append((x + bx, y + by, z0 + bz, br))
        TRUE_RIPENESS.append(ripeness)
        parts.append(f"""
        <visual name="berry{i}">
          <pose>{bx:.3f} {by:.3f} {z0 + bz:.3f} 0 0 0</pose>
          <geometry><sphere><radius>{br:.3f}</radius></sphere></geometry>
          <material>
            <ambient>{cr*0.62:.2f} {cg*0.62:.2f} {cb*0.62:.2f} 1</ambient>
            <diffuse>{cr:.2f} {cg:.2f} {cb:.2f} 1</diffuse>
            <!-- Was 0.8. Gazebo adds the specular term equally to R, G and B,
                 so a near-white gloss that strong rendered the LIT face of a
                 berry as (255, 106, 92): still red to the eye, but no longer
                 red by any channel-RATIO test, which is what blinded the
                 detector. 0.25 keeps the fruit shiny and keeps its hue. -->
            <specular>0.25 0.25 0.25 1</specular>
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


def distractor(name, x, y, rng):
    """A red object that is not a strawberry: a crate, a length of pipe, a
    dropped jacket -- scattered in the aisles. Deliberately absent from
    TRUE_BERRIES/TRUE_FOLIAGE: it must never be a positive label, and it must
    never occlude one either, since it sits in the empty aisle, not on a
    plant. This is the training set's answer to the colour-threshold
    detector's real failure mode (ml/README.md SS1): red things that are not
    fruit."""
    kind, base, spec, size_rng = rng.choice(DISTRACTOR_KINDS)
    jitter = lambda c: max(0.0, min(1.0, c + rng.uniform(-0.08, 0.08)))
    cr, cg, cb = (jitter(c) for c in base)
    yaw = rng.uniform(0.0, 6.283)
    if kind == "cylinder":
        r = rng.uniform(*size_rng)
        h = rng.uniform(0.10, 0.30)
        z = h / 2.0
        geometry = f"<cylinder><radius>{r:.3f}</radius><length>{h:.3f}</length></cylinder>"
    else:
        sx, sy, sz = (rng.uniform(*size_rng) for _ in range(3))
        z = sz / 2.0
        geometry = f"<box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>"
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.3f}</pose>
      <link name="link">
        <visual name="v">
          <geometry>{geometry}</geometry>
          <material>
            <ambient>{cr*0.7:.2f} {cg*0.7:.2f} {cb*0.7:.2f} 1</ambient>
            <diffuse>{cr:.2f} {cg:.2f} {cb:.2f} 1</diffuse>
            <specular>{spec:.2f} {spec:.2f} {spec:.2f} 1</specular>
          </material>
        </visual>
      </link>
    </model>"""


def build(rng: random.Random, domain: dict, seed: int) -> str:
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
            parts.append(plant(f"plant_{gi}_{pi}", x0 + pi * PLANT_SPACING, gy, seed))

    # Red, non-fruit distractors scattered in the two driving aisles (never
    # on a gutter, never registered as a berry or an occluder).
    for di in range(domain["n_distractors"]):
        ay = rng.choice((0.6, -0.6)) + rng.uniform(-0.28, 0.28)
        ax = rng.uniform(-ARENA_X_HALF + 0.7, ARENA_X_HALF - 0.7)
        parts.append(distractor(f"distractor_{di}", ax, ay, rng))

    # Lighting for this session: colour temperature, sun elevation/azimuth
    # and intensity, all domain-randomised (see sample_domain()).
    sun_rgb = kelvin_to_rgb(domain["temp_k"])
    sun_dx, sun_dy, sun_dz = sun_direction(domain["elev_deg"], domain["azim_deg"])
    intensity = domain["intensity"]
    sun_diffuse = tuple(c * intensity for c in sun_rgb)
    sun_specular = tuple(c * intensity * 0.4 for c in sun_rgb)
    amb = domain["ambient_scale"]
    # Overcast: the sun becomes a soft, shadowless skylight rather than a
    # directional source -- diffuse light from everywhere is exactly what
    # "overcast" looks like, and hard shadows would contradict the label.
    cast_shadows = "false" if domain["overcast"] else "true"
    fill_rgb = kelvin_to_rgb(11000)             # fixed cool sky fill
    fill_diffuse = tuple(c * 0.30 * amb for c in fill_rgb)
    scene_ambient = tuple(min(1.0, 0.50 * amb * (0.6 + 0.4 * c)) for c in sun_rgb)
    sky_base = (0.72, 0.84, 0.95)
    bg = tuple(min(1.0, 0.5 * s + 0.5 * c * amb) for s, c in zip(sky_base, sun_rgb))

    models = "".join(parts)
    return f"""<?xml version="1.0" ?>
<!-- Generated by scripts/make_world.py (do not edit by hand).
     Strawberry greenhouse digital twin (10 x 5 m). Geometry matches sim_node.py.
     Session domain: condition={domain['condition']} seed={seed}
     sun: {domain['temp_k']:.0f} K, elevation {domain['elev_deg']:.1f} deg,
     azimuth {domain['azim_deg']:.1f} deg, intensity {intensity:.2f},
     overcast={domain['overcast']}, distractors={domain['n_distractors']} -->
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
        <ambient_light>{scene_ambient[0]:.3f} {scene_ambient[1]:.3f} {scene_ambient[2]:.3f}</ambient_light>
        <background_color>{bg[0]:.3f} {bg[1]:.3f} {bg[2]:.3f}</background_color>
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
      <ambient>{scene_ambient[0]:.3f} {scene_ambient[1]:.3f} {scene_ambient[2]:.3f} 1</ambient>
      <background>{bg[0]:.3f} {bg[1]:.3f} {bg[2]:.3f} 1</background>
      <grid>false</grid>
      <sky></sky>
    </scene>

    <!-- Domain-randomised sun (colour temperature, elevation/azimuth and
         intensity; see the header comment above for this session's values)
         plus a fixed cool fill light so the glasshouse interior is never
         flat black on the shadow side. Overcast sessions drop the sun's
         shadow and lean on the fill instead, which is what a cloudy sky
         looks like: light from everywhere, no hard shadow. -->
    <light type="directional" name="sun">
      <cast_shadows>{cast_shadows}</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>{sun_diffuse[0]:.3f} {sun_diffuse[1]:.3f} {sun_diffuse[2]:.3f} 1</diffuse>
      <specular>{sun_specular[0]:.3f} {sun_specular[1]:.3f} {sun_specular[2]:.3f} 1</specular>
      <direction>{sun_dx:.4f} {sun_dy:.4f} {sun_dz:.4f}</direction>
    </light>
    <light type="directional" name="fill">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>{fill_diffuse[0]:.3f} {fill_diffuse[1]:.3f} {fill_diffuse[2]:.3f} 1</diffuse>
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


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Generate a domain-randomised worlds/greenhouse.sdf for "
                     "one capture session.")
    p.add_argument("--condition", choices=[*sorted(CONDITIONS), "random"],
                    default="random",
                    help="Lighting preset to sample this session's sun from "
                         "(default: pick one of the four at random, for "
                         "balanced coverage across many unattended sessions).")
    p.add_argument("--seed", type=int, default=None,
                    help="RNG seed. Fixes condition sampling, sun position, "
                         "fruit ripeness and distractor placement -- the same "
                         "seed with the same --condition reproduces this "
                         "world exactly. Default: a fresh, non-reproducible "
                         "seed from OS entropy.")
    p.add_argument("--session", default="",
                    help="Free-text session tag recorded in conditions.json "
                         "(e.g. 'cold_morning_01'). Purely informational.")
    p.add_argument("--out-dir", default=None,
                    help="Directory to write greenhouse.sdf, berries.yaml and "
                         "conditions.json into (default: ../worlds next to "
                         "this script).")
    return p.parse_args(argv)


def main(argv=None) -> None:
    args = parse_args(argv)
    seed = args.seed if args.seed is not None else random.SystemRandom().randrange(2**31)
    rng = random.Random(seed)
    domain = sample_domain(rng, args.condition)

    here = os.path.dirname(os.path.abspath(__file__))
    out_dir = os.path.normpath(args.out_dir or os.path.join(here, "..", "worlds"))
    os.makedirs(out_dir, exist_ok=True)

    text = build(rng, domain, seed)     # fills TRUE_BERRIES etc. as a side effect
    out = os.path.join(out_dir, "greenhouse.sdf")
    with open(out, "w") as f:
        f.write(text)

    # Ground-truth berry catalogue for youbot_slam/truth_monitor.
    ref = os.path.join(out_dir, "berries.yaml")
    with open(ref, "w") as f:
        f.write("# Ground-truth berry positions (x y z radius, metres), "
                "generated with the world.\n")
        f.write("# Used by youbot_slam/truth_monitor to score perception "
                "against reality.\n")
        f.write("berries:\n")
        for bx, by, bz, br in TRUE_BERRIES:
            f.write(f"  - [{bx:.4f}, {by:.4f}, {bz:.4f}, {br:.4f}]\n")
        f.write("# Foliage spheres, as occluders: a berry inside the camera "
                "frustum is only\n# visible if no leaf sits on the line of "
                "sight.\n")
        f.write("foliage:\n")
        for fx, fy, fz, fr in TRUE_FOLIAGE:
            f.write(f"  - [{fx:.4f}, {fy:.4f}, {fz:.4f}, {fr:.4f}]\n")
        # Continuous ripeness, one value per "berries:" row, same order. A
        # new section rather than a 5th tuple element -- see the comment on
        # TRUE_RIPENESS for why -- so it is additive, not a schema change.
        f.write("# Continuous ripeness in [0, 1] (0 = unripe green, 1 = "
                "fully ripe red), one entry\n# per berries: row above, same "
                "order. Not consumed by truth_monitor or berry_view; for "
                "the\n# ml/ training pipeline once it wants a ripe/unripe "
                "class split.\n")
        f.write("ripeness:\n")
        for rp in TRUE_RIPENESS:
            f.write(f"  - {rp:.4f}\n")
    print(f"wrote {ref} ({len(TRUE_BERRIES)} berries, "
          f"{len(TRUE_FOLIAGE)} leaves)")
    print(f"wrote {out}")

    # Session provenance: what a colleague running 4 capture sessions needs
    # to log against the resulting dataset, and what dataset_capture's
    # /dataset/conditions message (see ml/README.md SS2.1) should carry.
    cond_path = os.path.join(out_dir, "conditions.json")
    summary = {
        "session": args.session,
        "seed": seed,
        "condition": domain["condition"],
        "sun_color_temp_k": round(domain["temp_k"], 1),
        "sun_elevation_deg": round(domain["elev_deg"], 2),
        "sun_azimuth_deg": round(domain["azim_deg"], 2),
        "sun_intensity": round(domain["intensity"], 3),
        "ambient_scale": round(domain["ambient_scale"], 3),
        "overcast": domain["overcast"],
        "n_berries": len(TRUE_BERRIES),
        "n_distractors": domain["n_distractors"],
    }
    with open(cond_path, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"wrote {cond_path}")
    print("conditions: " + ", ".join(f"{k}={v}" for k, v in summary.items()))


if __name__ == "__main__":
    main()
