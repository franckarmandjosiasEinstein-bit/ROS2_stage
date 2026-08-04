#!/usr/bin/env python3
"""Generate worlds/greenhouse.sdf for Gazebo Harmonic.

The geometry mirrors the headless sim (sim_node.py) so behaviour is identical,
but here it is a real 3D world the gpu_lidar ray-casts against and RViz/Gazebo
render with colour, lighting and textures:

    Footprint  10 m (X) x 5 m (Y), origin at centre, walls 2.5 m high.
    Gutters    3 x (8.0 x 0.4 x 0.8) at Y = -1.2, 0, +1.2, spanning X[-4,4].
    Aisles     0.8 m between gutters (Y = +/-0.6); open cross-corridors at X ends.
    Plants     dense rows of strawberry plants on each gutter top: brown pot,
               bushy green foliage and strawberries across the ripeness
               continuum (the harvest targets). Nothing is placed in the
               driving aisles except, optionally, red distractor props.

DOMAIN RANDOMISATION

Everything a real greenhouse visit varies by time of day or season is a CLI
flag here, not a constant: colour temperature, sun direction/elevation and
intensity, cloud cover (softens/kills the directional shadow), fruit
maturity (continuous, not ripe/unripe as a switch), and a scatter of
non-fruit red distractor props (the false-positive source called out in
ml/README.md). `--preset` sets all of them at once to the four lighting
blocks the ml/README data spec asks for; any flag after `--preset` on the
command line overrides just that one value.

Run:  python3 scripts/make_world.py                      (unchanged default)
      python3 scripts/make_world.py --preset cold_morning --seed 1
      python3 scripts/make_world.py --list-presets
"""

from __future__ import annotations

import argparse
import math
import os
import random
from dataclasses import dataclass, replace

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

AISLES_Y = (0.6, -0.6)
AISLE_X_HALF = ARENA_X_HALF - 0.65   # keep clear of the end corridors


# =====================================================================
#  Domain-randomisation parameters
# =====================================================================
@dataclass(frozen=True)
class DomainParams:
    seed: int = 0
    # Colour temperature of the sun, kelvin. ~2000-3000 = candle/sunrise,
    # ~5500-6500 = neutral daylight, ~7500-10000 = overcast/blue-sky shade.
    color_temp_k: float = 5500.0
    # Sun position in the sky: elevation 90 = straight overhead (noon),
    # elevation near 0 = grazing the horizon (dawn/dusk). Azimuth is the
    # compass direction the sun sits in, degrees, 0 = +X.
    sun_elevation_deg: float = 55.0
    sun_azimuth_deg: float = 200.0
    sun_intensity: float = 1.0          # multiplier on the base diffuse
    # 0 = clear sky (hard directional shadow). 1 = fully overcast (the sun
    # term nearly vanishes, skylight/fill dominates, no shadows).
    cloud_cover: float = 0.0
    # Fruit maturity is a continuum, not ripe/unripe as a coin flip: each
    # berry draws a ripeness in [0, 1] from a distribution centred on
    # ripeness_mean with spread ripeness_spread (both clamped to [0, 1]).
    ripeness_mean: float = 0.72
    ripeness_spread: float = 0.30
    # Non-fruit red props scattered in the driving aisles -- crates, tools,
    # pipe offcuts -- close enough in hue to a ripe berry to be exactly the
    # false-positive case the colour-threshold detector fails on.
    n_red_distractors: int = 6


PRESETS: dict[str, DomainParams] = {
    # 4 lighting blocks called for by ml/README.md sec. 1: "Morning, midday,
    # late afternoon, overcast -- the colour-temperature failure -- 4 blocks,
    # balanced."
    "cold_morning": DomainParams(
        color_temp_k=9500.0, sun_elevation_deg=12.0, sun_azimuth_deg=95.0,
        sun_intensity=0.55, cloud_cover=0.10,
        ripeness_mean=0.55, ripeness_spread=0.35, n_red_distractors=6),
    "neutral_midday": DomainParams(
        color_temp_k=5800.0, sun_elevation_deg=80.0, sun_azimuth_deg=180.0,
        sun_intensity=1.05, cloud_cover=0.0,
        ripeness_mean=0.75, ripeness_spread=0.28, n_red_distractors=6),
    "warm_late_afternoon": DomainParams(
        color_temp_k=2600.0, sun_elevation_deg=9.0, sun_azimuth_deg=265.0,
        sun_intensity=0.65, cloud_cover=0.05,
        ripeness_mean=0.82, ripeness_spread=0.22, n_red_distractors=6),
    "overcast": DomainParams(
        color_temp_k=7200.0, sun_elevation_deg=45.0, sun_azimuth_deg=150.0,
        sun_intensity=0.35, cloud_cover=0.85,
        ripeness_mean=0.65, ripeness_spread=0.32, n_red_distractors=6),
}


def kelvin_to_rgb(k: float) -> tuple[float, float, float]:
    """Approximate blackbody colour temperature -> normalised (r, g, b).

    Tanner Helland's fit, clamped to Gazebo's usual 1000-12000 K working
    range. Good enough for tinting a light's <diffuse>, not colourimetry.
    """
    t = max(1000.0, min(12000.0, k)) / 100.0

    if t <= 66.0:
        r = 255.0
    else:
        r = 329.698727446 * ((t - 60.0) ** -0.1332047592)
    r = max(0.0, min(255.0, r))

    if t <= 66.0:
        g = 99.4708025861 * math.log(t) - 161.1195681661
    else:
        g = 288.1221695283 * ((t - 60.0) ** -0.0755148492)
    g = max(0.0, min(255.0, g))

    if t >= 66.0:
        b = 255.0
    elif t <= 19.0:
        b = 0.0
    else:
        b = 138.5177312231 * math.log(t - 10.0) - 305.0447927307
    b = max(0.0, min(255.0, b))

    return r / 255.0, g / 255.0, b / 255.0


def sun_direction(elevation_deg: float, azimuth_deg: float) -> tuple[float, float, float]:
    """Unit direction the light TRAVELS (Gazebo convention), from a sky
    position given as elevation above the horizon and compass azimuth."""
    el = math.radians(elevation_deg)
    az = math.radians(azimuth_deg)
    x = -math.cos(el) * math.cos(az)
    y = -math.cos(el) * math.sin(az)
    z = -math.sin(el)
    n = math.sqrt(x * x + y * y + z * z) or 1.0
    return x / n, y / n, z / n


def _mix(a: float, b: float, t: float) -> float:
    return a + (b - a) * max(0.0, min(1.0, t))


def ripeness_to_rgb(ripeness: float) -> tuple[float, float, float]:
    """Continuous unripe(green) -> ripe(red) berry colour, ripeness in [0,1].

    Follows a real strawberry's arc: green, then a pale green-white, then
    orange-pink, then deep glossy red -- not a straight green->red lerp,
    which produces a muddy brown through the middle of the range.
    """
    stops = (
        (0.00, (0.20, 0.42, 0.12)),   # hard green
        (0.35, (0.55, 0.58, 0.20)),   # turning, green-yellow
        (0.60, (0.85, 0.45, 0.20)),   # pink-orange
        (1.00, (0.95, 0.08, 0.09)),   # fully ripe red
    )
    r = max(0.0, min(1.0, ripeness))
    for (t0, c0), (t1, c1) in zip(stops, stops[1:]):
        if r <= t1 or (t0, c0) == stops[-2]:
            span = (t1 - t0) or 1.0
            local = (r - t0) / span
            return tuple(_mix(a, b, local) for a, b in zip(c0, c1))
    return stops[-1][1]


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
# Kept as a PARALLEL list rather than appended onto each berry tuple: several
# readers (truth_monitor._berries_in_view's `for i, (bx, by, bz, br) in
# enumerate(...)`) unpack berries.yaml's `berries:` section as exact 4-tuples,
# so widening that schema breaks perception scoring. A sibling section is
# additive and every existing reader is untouched.
TRUE_RIPENESS = []
# Every foliage sphere, as an OCCLUDER. A berry being inside the camera frustum
# does not mean the camera can see it: half the fruit on a plant hangs behind
# that plant's own leaves. Without this, "berries truly in view" counts fruit
# no camera could ever report and every run reads as a perception failure.
TRUE_FOLIAGE = []


def plant(name, x, y, dom: DomainParams):
    """A realistic-looking strawberry plant sitting on the gutter top: a bushy
    cluster of green leaves and strawberries spanning the ripeness continuum,
    each plant slightly different (deterministic per-name randomness, so every
    rebuild with the same seed is identical). Purely visual (well above the
    0.20 m lidar plane, so it never clutters the map) -- these fruited plants
    are the harvest targets."""
    rng = random.Random(f"{dom.seed}:{name}")   # deterministic per (seed, plant)
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
    # Strawberries hanging around the bush: ripeness is a continuum, drawn per
    # berry around dom.ripeness_mean, not a fixed "always ripe" red.
    n_berry = rng.randint(4, 6)
    for i in range(n_berry):
        ang = rng.uniform(0, 6.283)
        rad = rng.uniform(0.10, 0.14)
        bx = rad * math.cos(ang)
        by = rad * 0.8 * math.sin(ang)
        bz = rng.uniform(0.06, 0.12)
        ripeness = max(0.0, min(1.0, rng.gauss(dom.ripeness_mean, dom.ripeness_spread)))
        br = rng.uniform(0.028, 0.036) * (0.82 + 0.18 * ripeness)  # ripe fruit swells slightly
        cr, cg, cb = ripeness_to_rgb(ripeness)
        TRUE_BERRIES.append((x + bx, y + by, z0 + bz, br))
        TRUE_RIPENESS.append(ripeness)
        parts.append(f"""
        <visual name="berry{i}">
          <pose>{bx:.3f} {by:.3f} {z0 + bz:.3f} 0 0 0</pose>
          <geometry><sphere><radius>{br:.3f}</radius></sphere></geometry>
          <material>
            <ambient>{cr*0.62:.3f} {cg*0.62:.3f} {cb*0.62:.3f} 1</ambient>
            <diffuse>{cr:.3f} {cg:.3f} {cb:.3f} 1</diffuse>
            <!-- Was 0.8 flat white. Gazebo adds the specular term equally to
                 R, G and B, so a near-white gloss that strong rendered the
                 LIT face of a ripe berry as (255, 106, 92): still red to the
                 eye, but no longer red by any channel-RATIO test, which is
                 what blinded the detector. 0.25 keeps the fruit shiny and
                 keeps it on-hue; unripe (greener) berries get an even lower
                 gloss since unripe skin is less waxy. -->
            <specular>{0.14 + 0.11*ripeness:.3f} {0.14 + 0.11*ripeness:.3f} {0.14 + 0.11*ripeness:.3f} 1</specular>
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


# Non-fruit red props: shapes and hues chosen to be exactly what a
# channel-ratio "is it red" test cannot tell from a strawberry, and what
# ml/README.md's data spec asks for ("Red objects that are *not* fruit:
# pipes, tools, crates, clothing -- this is where false positives come
# from -- >= 15% of frames").
_DISTRACTOR_KINDS = ("crate", "pipe", "tool")


def distractor(name, x, y, kind, rng):
    """One red distractor prop, static, resting in a driving aisle."""
    hue = rng.uniform(0.75, 1.0)   # ripe-adjacent red, on purpose
    r, g, b = hue, 0.06 * hue, 0.08 * hue
    yaw = rng.uniform(0, 6.283)

    if kind == "crate":
        sx, sy, sz = rng.uniform(0.22, 0.30), rng.uniform(0.16, 0.22), rng.uniform(0.14, 0.20)
        geom = f"<box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>"
        z = sz / 2.0
    elif kind == "pipe":
        radius, length = rng.uniform(0.02, 0.035), rng.uniform(0.35, 0.65)
        geom = f"<cylinder><radius>{radius:.3f}</radius><length>{length:.3f}</length></cylinder>"
        z = radius            # lying on its side
        yaw = 1.5708           # horizontal, across the aisle
    else:  # tool: a thin red-handled shape (approximated as a slim box)
        sx, sy, sz = rng.uniform(0.30, 0.45), 0.045, 0.045
        geom = f"<box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>"
        z = sz / 2.0

    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.3f}</pose>
      <link name="link">
        <collision name="c"><geometry>{geom}</geometry></collision>
        <visual name="v">
          <geometry>{geom}</geometry>
          <material>
            <ambient>{r*0.6:.3f} {g*0.6:.3f} {b*0.6:.3f} 1</ambient>
            <diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>
            <specular>0.35 0.35 0.35 1</specular>
          </material>
        </visual>
      </link>
    </model>"""


def build(dom: DomainParams) -> str:
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
            parts.append(plant(f"plant_{gi}_{pi}", x0 + pi * PLANT_SPACING, gy, dom))

    # Red distractor props, scattered in the driving aisles only -- never on
    # a gutter top, so they can never be confused with "on the plant".
    drng = random.Random(f"{dom.seed}:distractors")
    for di in range(dom.n_red_distractors):
        dy = AISLES_Y[di % len(AISLES_Y)] + drng.uniform(-0.18, 0.18)
        dx = drng.uniform(-AISLE_X_HALF, AISLE_X_HALF)
        kind = _DISTRACTOR_KINDS[di % len(_DISTRACTOR_KINDS)]
        parts.append(distractor(f"distractor_{di}", dx, dy, kind, drng))

    models = "".join(parts)

    # --- lighting, driven by the domain-randomisation params ---
    tr, tg, tb = kelvin_to_rgb(dom.color_temp_k)
    clear = max(0.0, 1.0 - dom.cloud_cover)
    sun_mag = 0.95 * dom.sun_intensity * clear
    sx, sy, sz = sun_direction(dom.sun_elevation_deg, dom.sun_azimuth_deg)
    sun_diffuse = (min(1.0, tr * sun_mag), min(1.0, tg * sun_mag), min(1.0, tb * sun_mag))
    sun_specular = tuple(0.42 * clear * c for c in (tr, tg, tb))

    # Fill/skylight: nearly colour-temperature-neutral, and it is what
    # dominates once cloud_cover pushes the sun term toward zero -- an
    # overcast sky is still lit, just with no single dominant direction.
    fill_mag = 0.30 + 0.55 * dom.cloud_cover
    fill_diffuse = (0.30 * fill_mag, 0.31 * fill_mag, 0.34 * fill_mag)

    scene_ambient = 0.42 + 0.22 * dom.cloud_cover
    bg = (0.72 - 0.10 * dom.cloud_cover, 0.84 - 0.06 * dom.cloud_cover, 0.95 - 0.02 * dom.cloud_cover)
    cast_shadows = "false" if dom.cloud_cover > 0.7 else "true"

    return f"""<?xml version="1.0" ?>
<!-- Generated by scripts/make_world.py (do not edit by hand).
     Strawberry greenhouse digital twin (10 x 5 m). Geometry matches sim_node.py.
     Domain randomisation: color_temp_k={dom.color_temp_k:.0f} sun_elevation_deg={dom.sun_elevation_deg:.1f}
     sun_azimuth_deg={dom.sun_azimuth_deg:.1f} sun_intensity={dom.sun_intensity:.2f}
     cloud_cover={dom.cloud_cover:.2f} ripeness_mean={dom.ripeness_mean:.2f}
     ripeness_spread={dom.ripeness_spread:.2f} n_red_distractors={dom.n_red_distractors} seed={dom.seed} -->
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
      <ambient>{scene_ambient:.3f} {scene_ambient:.3f} {scene_ambient*0.96:.3f} 1</ambient>
      <background>{bg[0]:.3f} {bg[1]:.3f} {bg[2]:.3f} 1</background>
      <grid>false</grid>
      <sky></sky>
    </scene>

    <!-- Sun: direction/elevation/intensity/colour temperature all come from
         DomainParams so a session can be "cold morning", "warm late
         afternoon", etc. purely from CLI flags; see PRESETS. -->
    <light type="directional" name="sun">
      <cast_shadows>{cast_shadows}</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>{sun_diffuse[0]:.3f} {sun_diffuse[1]:.3f} {sun_diffuse[2]:.3f} 1</diffuse>
      <specular>{sun_specular[0]:.3f} {sun_specular[1]:.3f} {sun_specular[2]:.3f} 1</specular>
      <direction>{sx:.4f} {sy:.4f} {sz:.4f}</direction>
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


def _parse_args(argv=None) -> tuple[DomainParams, argparse.Namespace]:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--preset", choices=sorted(PRESETS), default=None,
                   help="Set all domain-randomisation params to one of the "
                        "four ml/README.md lighting blocks; any flag below "
                        "still overrides just that one value.")
    p.add_argument("--list-presets", action="store_true",
                   help="Print the resolved parameters for every preset and exit.")
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--color-temp-k", type=float, default=None)
    p.add_argument("--sun-elevation-deg", type=float, default=None)
    p.add_argument("--sun-azimuth-deg", type=float, default=None)
    p.add_argument("--sun-intensity", type=float, default=None)
    p.add_argument("--cloud-cover", type=float, default=None,
                   help="0 = clear sky, 1 = fully overcast.")
    p.add_argument("--ripeness-mean", type=float, default=None)
    p.add_argument("--ripeness-spread", type=float, default=None)
    p.add_argument("--n-red-distractors", type=int, default=None)
    p.add_argument("--out-dir", default=None,
                   help="Override the worlds/ output directory (default: "
                        "../worlds relative to this script).")
    args = p.parse_args(argv)

    dom = PRESETS[args.preset] if args.preset else DomainParams()
    overrides = {
        k: v for k, v in dict(
            seed=args.seed, color_temp_k=args.color_temp_k,
            sun_elevation_deg=args.sun_elevation_deg,
            sun_azimuth_deg=args.sun_azimuth_deg,
            sun_intensity=args.sun_intensity, cloud_cover=args.cloud_cover,
            ripeness_mean=args.ripeness_mean,
            ripeness_spread=args.ripeness_spread,
            n_red_distractors=args.n_red_distractors,
        ).items() if v is not None
    }
    dom = replace(dom, **overrides)
    return dom, args


def main(argv=None) -> None:
    dom, args = _parse_args(argv)

    if args.list_presets:
        for name, p in sorted(PRESETS.items()):
            print(f"{name}: {p}")
        return

    here = os.path.dirname(os.path.abspath(__file__))
    out_dir = os.path.normpath(args.out_dir or os.path.join(here, "..", "worlds"))
    os.makedirs(out_dir, exist_ok=True)

    out = os.path.join(out_dir, "greenhouse.sdf")
    text = build(dom)                   # fills TRUE_BERRIES/TRUE_RIPENESS/TRUE_FOLIAGE as a side effect
    with open(out, "w") as f:
        f.write(text)

    # Ground-truth berry catalogue for youbot_slam/truth_monitor. The
    # `berries:` section keeps its original (x, y, z, radius) 4-tuple schema
    # unchanged -- truth_monitor unpacks it positionally and a 5th field
    # would break that -- so ripeness rides along as a same-order sibling
    # section instead of widening each berry entry.
    ref = os.path.join(out_dir, "berries.yaml")
    with open(ref, "w") as f:
        f.write("# Ground-truth berry positions (x y z radius, metres), "
                "generated with the world.\n")
        f.write("# Used by youbot_slam/truth_monitor to score perception "
                "against reality.\n")
        f.write("berries:\n")
        for bx, by, bz, br in TRUE_BERRIES:
            f.write(f"  - [{bx:.4f}, {by:.4f}, {bz:.4f}, {br:.4f}]\n")
        f.write("# Continuous ripeness in [0, 1], SAME ORDER as `berries:` "
                "above (index i of one is index i of the other). Not yet "
                "consumed by dataset_capture (still writes class 0 for "
                "every box); the values are here so a ripe/unripe split can "
                "be turned on without regenerating the world.\n")
        f.write("ripeness:\n")
        for rp in TRUE_RIPENESS:
            f.write(f"  - {rp:.4f}\n")
        f.write("# Foliage spheres, as occluders: a berry inside the camera "
                "frustum is only\n# visible if no leaf sits on the line of "
                "sight.\n")
        f.write("foliage:\n")
        for fx, fy, fz, fr in TRUE_FOLIAGE:
            f.write(f"  - [{fx:.4f}, {fy:.4f}, {fz:.4f}, {fr:.4f}]\n")

    meta = os.path.join(out_dir, "domain_params.yaml")
    with open(meta, "w") as f:
        f.write("# Exact domain-randomisation parameters used for this "
                "world -- written by make_world.py so a capture session can "
                "record what it varied (ml/README.md sec 2.1).\n")
        f.write(f"preset: {args.preset!r}\n")
        for k, v in dom.__dict__.items():
            f.write(f"{k}: {v}\n")

    print(f"wrote {ref} ({len(TRUE_BERRIES)} berries, "
          f"{len(TRUE_FOLIAGE)} leaves, ripeness mean "
          f"{ (sum(TRUE_RIPENESS)/len(TRUE_RIPENESS)) if TRUE_RIPENESS else 0.0:.2f})")
    print(f"wrote {out}")
    print(f"wrote {meta}")


if __name__ == "__main__":
    main()
