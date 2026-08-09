"""plants -- swap the Phase B sphere blobs for a real strawberry mesh.

WHAT THE PHASE B PLANTS ARE

Clusters of coloured spheres. They were built to be a lidar obstacle and a
patch of red for a fruit detector, and at that they work. They do not look
like strawberry plants, and a demonstration is partly a visual argument.

WHY THIS IS A SEPARATE STEP AND NOT PART OF make_world

Because the mesh is an ASSET and the crosses are DERIVED DATA. The crosses
come out of agri/catalogue.py and cannot be wrong without a check going red;
the mesh comes from outside the project, has a licence, a polygon count and
an origin convention, and none of those are things this repository can
assert. Keeping the two generators apart means a missing or unusable mesh
degrades to "the world still builds, with the old plants" rather than to a
world that will not build at all.

    python3 -m agri.world.make_plants --mesh meshes/strawberry.dae

THE TWO CONSTRAINTS THAT DECIDE WHETHER A GIVEN MESH IS USABLE

1.  COLLISION MUST STAY PRIMITIVE. A 50 000-triangle mesh used as a
    collision shape asks the physics engine to test 50 000 triangles per
    contact query, twenty-four times over, at 1 ms steps. The visual is
    replaced; the collision cylinder is not. This is standard practice and
    it is also the difference between a simulation that runs and one that
    does not.

2.  THE RENDER BUDGET IS REAL. Twenty-four plants at 50 k triangles is
    1.2 M triangles in a scene that also runs a GPU lidar, two cameras and
    physics. On a laptop with integrated graphics that is where the frame
    rate goes. --every-nth exists for exactly this: place the mesh on a
    subset and leave the rest as blobs, which keeps the argument (these are
    strawberry plants) at a fraction of the cost.

WHAT THIS DOES NOT TOUCH

The gutters, the walls, the crosses, the plant POSITIONS. A plant's pose is
read out of the existing world and reused verbatim, so the catalogue's idea
of where the plants are cannot drift from where they are drawn. The suite
asserts that after a swap the plant positions still match the catalogue.
"""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

#: Where a plant's mesh origin sits relative to the model origin. The Phase B
#: models put the plant crown near z = 0.95 (the gutter top is 0.80 m); a
#: mesh exported with its origin at the root ball needs lifting to the same
#: place. Overridable, because no two asset stores agree on this.
DEFAULT_Z = 0.80

#: A strawberry plant is 20-30 cm across and about 25 cm tall. Most stores
#: export in centimetres or in arbitrary units; --scale is how that is
#: reconciled without editing the file.
DEFAULT_SCALE = 1.0

#: Kept from the Phase B plant: the lidar must still see something solid at
#: the gutter, and the brake reasons about it.
COLLISION = """        <collision name="stem">
          <pose>0 0 {cz:.3f} 0 0 0</pose>
          <geometry><cylinder><radius>0.11</radius><length>0.22</length></cylinder></geometry>
        </collision>"""


def plant_sdf(name: str, pose: str, uri: str, scale: float, z: float,
              yaw: float) -> str:
    """One plant model with a mesh visual and a primitive collision.

    `yaw` varies per plant so twenty-four instances of one asset do not read
    as twenty-four copies of one asset. It costs nothing -- the mesh is
    loaded once and instanced -- and it is the cheapest thing that makes a
    generated row look grown rather than stamped.
    """
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <visual name="plant">
          <pose>0 0 {z:.3f} 0 0 {yaw:.4f}</pose>
          <geometry>
            <mesh>
              <uri>{uri}</uri>
              <scale>{scale:.4f} {scale:.4f} {scale:.4f}</scale>
            </mesh>
          </geometry>
        </visual>
{COLLISION.format(cz=z + 0.11)}
      </link>
    </model>
"""


def swap(sdf: str, uri: str, scale: float, z: float,
         every_nth: int = 1) -> tuple[str, int]:
    """Replace plant models in `sdf` with the mesh. Returns (sdf, replaced).

    Matched on the model NAME rather than on its contents: the Phase B
    plants are sphere clusters today and the point of this function is that
    they need not stay that way.
    """
    out, replaced, seen = [], 0, 0
    pos = 0
    pattern = re.compile(r'    <model name="(plant_\d+_\d+)">.*?</model>\n',
                         re.S)
    for m in pattern.finditer(sdf):
        out.append(sdf[pos:m.start()])
        pos = m.end()
        name = m.group(1)
        pose_m = re.search(r"<pose>([^<]+)</pose>", m.group(0))
        if pose_m is None:                    # no pose: leave it untouched
            out.append(m.group(0))
            continue
        if seen % max(1, every_nth):
            out.append(m.group(0))            # this one keeps its blobs
            seen += 1
            continue
        seen += 1
        # A deterministic pseudo-yaw from the name, so regenerating twice
        # gives the same world and a diff of two runs is empty.
        yaw = (sum(ord(c) for c in name) % 360) * 3.14159265 / 180.0
        out.append(plant_sdf(name, pose_m.group(1).strip(), uri, scale, z,
                             yaw))
        replaced += 1
    out.append(sdf[pos:])
    return "".join(out), replaced


def build(world: Path, mesh: Path, scale: float, z: float,
          every_nth: int, uri: str | None = None) -> str:
    if not world.exists():
        raise FileNotFoundError(
            f"{world} does not exist. Generate it first:\n"
            "    python3 -m agri.world.make_world")
    if uri is None:
        if not mesh.exists():
            raise FileNotFoundError(
                f"{mesh} does not exist.\n"
                "Put the strawberry mesh there, or pass --uri to reference a\n"
                "model:// or package:// path Gazebo already resolves.")
        # An absolute file:// URI is the only form that works regardless of
        # where gz is launched from and of what is on GZ_SIM_RESOURCE_PATH.
        # Less portable than model://, and portable enough for a mesh that
        # lives in this repository.
        uri = "file://" + str(mesh.resolve())

    sdf = world.read_text()
    if "<mesh>" in sdf and "plant" in sdf.split("<mesh>", 1)[0][-400:]:
        raise ValueError(
            f"{world} already has mesh plants. Regenerate the world first:\n"
            "    python3 -m agri.world.make_world --force")
    swapped, n = swap(sdf, uri, scale, z, every_nth)
    if n == 0:
        raise ValueError(
            f"no plant models found in {world}. Expected models named "
            "plant_<row>_<index>; has the Phase B world changed?")
    world.write_text(swapped)
    return (f"{n} plant(s) now use {Path(uri).name}\n"
            f"  scale {scale}, crown at z = {z:.2f} m, "
            f"every {every_nth} plant(s)\n"
            f"  collision left as a primitive cylinder -- a mesh collision "
            f"would cost the physics engine every triangle, 24 times, "
            f"every millisecond")


# =====================================================================
#  READING A MESH, SO THE SCALE IS MEASURED RATHER THAN GUESSED
# =====================================================================
#  An asset arrives in whatever units its author used, with its origin
#  wherever their modeller put it. Asking the operator for --scale and --z
#  means two numbers found by launching Gazebo, squinting, and launching it
#  again. Both are readable straight out of a binary STL, so they are.
# ---------------------------------------------------------------------
#: How wide a strawberry plant is, across the leaves. The one number here
#: that comes from the crop and not from a file: a mature plant spans about
#: 25-30 cm, and every mesh is scaled to match it.
PLANT_WIDTH_M = 0.27


def stl_bounds(path: Path) -> tuple[tuple[float, float, float],
                                    tuple[float, float, float]]:
    """(min, max) corner of a binary STL, in the file's own units.

    Binary only, deliberately. An ASCII STL is detected and refused with
    the conversion command rather than mis-parsed: the two formats share an
    extension, and a silent misread here would place every plant somewhere
    plausible and wrong.
    """
    import struct                                        # noqa: PLC0415

    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"{path} is too short to be an STL")
    if data[:5] == b"solid" and b"facet" in data[:2000]:
        raise ValueError(
            f"{path} is an ASCII STL and this reads binary ones.\n"
            "    Convert it:  python3 -c \"import trimesh;"
            "trimesh.load('in.stl').export('out.stl')\"\n"
            "    or re-export as binary from whatever produced it.")
    n = struct.unpack("<I", data[80:84])[0]
    if len(data) < 84 + 50 * n:
        raise ValueError(f"{path} claims {n} triangles and is too short "
                         "for them; the file is truncated")
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    off = 84
    for _ in range(n):
        v = struct.unpack("<12fH", data[off:off + 50])
        off += 50
        for k in range(3):
            for c in range(3):
                q = v[3 + k * 3 + c]
                if q < lo[c]:
                    lo[c] = q
                if q > hi[c]:
                    hi[c] = q
    return tuple(lo), tuple(hi)


def glb_bounds(path: Path) -> tuple[tuple[float, float, float],
                                    tuple[float, float, float]]:
    """(min, max) of a binary glTF, read from its accessor metadata.

    No mesh data is decoded: every POSITION accessor in a glTF is REQUIRED
    to carry its own min and max, so the bounding box is in the JSON chunk
    and reading it is a dictionary lookup rather than a parse of several
    megabytes of vertices.
    """
    import json                                          # noqa: PLC0415
    import struct                                        # noqa: PLC0415

    d = path.read_bytes()
    if len(d) < 12 or struct.unpack("<I", d[:4])[0] != 0x46546C67:
        raise ValueError(
            f"{path} is not a binary glTF (.glb).\n"
            "    A text .gltf keeps its buffers in separate files; export "
            "or convert to .glb so the mesh and its textures travel as one.")
    total = struct.unpack("<I", d[8:12])[0]
    off, js = 12, None
    while off < total and off + 8 <= len(d):
        ln, kind = struct.unpack("<II", d[off:off + 8])
        if kind == 0x4E4F534A:                           # 'JSON'
            js = json.loads(d[off + 8:off + 8 + ln])
            break
        off += 8 + ln
    if js is None:
        raise ValueError(f"{path} has no JSON chunk")
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    for mesh in js.get("meshes", []):
        for prim in mesh.get("primitives", []):
            acc = js["accessors"][prim["attributes"]["POSITION"]]
            if "min" not in acc or "max" not in acc:
                raise ValueError(f"{path} has a POSITION accessor with no "
                                 "min/max; the file is not spec-compliant")
            for c in range(3):
                lo[c] = min(lo[c], acc["min"][c])
                hi[c] = max(hi[c], acc["max"][c])
    if lo[0] == float("inf"):
        raise ValueError(f"{path} contains no mesh")
    return tuple(lo), tuple(hi)


def glb_is_self_contained(path: Path) -> bool:
    """Does this .glb carry the images its material asks for?

    THE FAILURE THIS CATCHES IS SILENT. A glTF can reference a
    baseColorTexture and ship none of the image data -- three of the four
    plants offered to this project did exactly that. Gazebo loads the mesh,
    finds no texture, and renders it white. Nothing errors; the greenhouse
    just fills with pale plants and the cause is two levels down in a
    binary file.
    """
    import json                                          # noqa: PLC0415
    import struct                                        # noqa: PLC0415

    d = path.read_bytes()
    total = struct.unpack("<I", d[8:12])[0]
    off, js = 12, None
    while off < total and off + 8 <= len(d):
        ln, kind = struct.unpack("<II", d[off:off + 8])
        if kind == 0x4E4F534A:
            js = json.loads(d[off + 8:off + 8 + ln])
            break
        off += 8 + ln
    if js is None:
        return False
    wants = any("baseColorTexture" in m.get("pbrMetallicRoughness", {})
                for m in js.get("materials", []))
    return (not wants) or bool(js.get("images"))


def mesh_bounds(path: Path):
    """Bounding box of whatever format this is, by extension."""
    if path.suffix.lower() == ".glb":
        return glb_bounds(path)
    return stl_bounds(path)


def triangle_count(path: Path) -> int:
    import json                                          # noqa: PLC0415
    import struct                                        # noqa: PLC0415

    d = path.read_bytes()
    if path.suffix.lower() != ".glb":
        return struct.unpack("<I", d[80:84])[0]
    total = struct.unpack("<I", d[8:12])[0]
    off, js = 12, None
    while off < total and off + 8 <= len(d):
        ln, kind = struct.unpack("<II", d[off:off + 8])
        if kind == 0x4E4F534A:
            js = json.loads(d[off + 8:off + 8 + ln])
            break
        off += 8 + ln
    n = 0
    for mesh in (js or {}).get("meshes", []):
        for prim in mesh.get("primitives", []):
            if "indices" in prim:
                n += js["accessors"][prim["indices"]]["count"] // 3
            else:
                n += js["accessors"][
                    prim["attributes"]["POSITION"]]["count"] // 3
    return n


#: WHICH WAY IS UP, PER FORMAT. This is not a detail.
#:
#: glTF fixes it in the specification: +Y is up, +Z is forward. STL fixes
#: nothing, and the two supplied here happen to be Z-up like the rest of
#: this project.
#:
#: fit() assumed Z for both, because the STLs came first and the GLBs were
#: added without re-asking. The result was every plant lying on its side in
#: the greenhouse -- which is hard to see in a render of a rosette, and was
#: finally given away by the one mesh with a base plate baked in: the plate
#: stood up as a dark vertical wall in the middle of every photograph, and
#: was reported as "why are the plates arranged like that".
#:
#: Gazebo does not rotate a glTF for you. The +90 degree roll that puts a
#: Y-up mesh upright is written into the pose by mesh_plant_sdf().
UP_AXIS = {".glb": 1, ".gltf": 1, ".stl": 2, ".dae": 2}


def up_axis(path: Path) -> int:
    """0, 1 or 2 -- the index of this format's vertical axis."""
    return UP_AXIS.get(path.suffix.lower(), 2)


def needs_roll(path: Path) -> bool:
    """True when the mesh has to be rolled +90 deg to stand up in Gazebo."""
    return up_axis(path) == 1


def fit(path: Path, width_m: float = PLANT_WIDTH_M
        ) -> tuple[float, float, int]:
    """(scale, lift, triangles) to stand this mesh on the gutter top.

    scale   makes the plant `width_m` across, whatever units it came in
    lift    raises it so its LOWEST point rests on the surface, rather
            than half-buried -- these meshes are centred on their own
            origin, so without this every plant sinks by half its height

    Both are measured against the format's own up axis (see UP_AXIS), not
    against Z. Getting that wrong scales the plant by its height instead of
    its width and lifts it by the wrong number, on top of leaving it lying
    down.
    """
    lo, hi = mesh_bounds(path)
    up = up_axis(path)
    across = [hi[a] - lo[a] for a in range(3) if a != up]
    span = max(across)
    if span <= 0:
        raise ValueError(f"{path} has no width; is it a valid mesh?")
    scale = width_m / span
    return scale, -lo[up] * scale, triangle_count(path)


def fitted_height(path: Path, width_m: float = PLANT_WIDTH_M) -> float:
    """How tall this mesh stands once fitted, in metres."""
    lo, hi = mesh_bounds(path)
    up = up_axis(path)
    return (hi[up] - lo[up]) * fit(path, width_m)[0]


# =====================================================================
#  PROCEDURAL STRAWBERRY PLANT
# =====================================================================
#  Built from primitives, because a sculpted asset costs money and this
#  costs nothing. It will not be mistaken for the photogrammetry models
#  the asset stores sell: there are no leaf veins, no seed dimples and no
#  subsurface scattering, and there cannot be without textures.
#
#  What it does buy, against the sphere blobs it replaces, is the four
#  things that make a plant read AS a strawberry plant at demonstration
#  distance:
#
#    a low rosette      strawberries grow from a crown, not on a stem
#    trifoliate leaves  three leaflets per stalk. This is the single most
#                       recognisable thing about the species and costs
#                       three ellipsoids.
#    hanging fruit      on arching trusses BELOW the leaf canopy, which is
#                       where strawberries actually hang and where every
#                       stylised drawing of one puts them wrong
#    ripeness spread    red, blush and green on the same plant, because a
#                       real one carries all three at once
#
#  Roughly 40 primitives per plant against 50 000 triangles for a bought
#  mesh, and no texture files to lose.
# ---------------------------------------------------------------------
LEAF_GREEN = ("<ambient>0.05 0.20 0.04 1</ambient>"
              "<diffuse>0.13 0.45 0.11 1</diffuse>"
              "<specular>0.08 0.12 0.06 1</specular>")
STEM_GREEN = ("<ambient>0.06 0.18 0.05 1</ambient>"
              "<diffuse>0.16 0.38 0.13 1</diffuse>")
#: Three ripeness stages. A plant carries all of them at once, so the row
#: does not read as a single flat colour -- and so the fruit detector of
#: Phase A/B would have something to be wrong about (Sec. extension).
FRUIT = {
    "ripe":  "<ambient>0.42 0.02 0.03 1</ambient>"
             "<diffuse>0.88 0.07 0.09 1</diffuse>"
             "<specular>0.5 0.3 0.3 1</specular>",
    "blush": "<ambient>0.40 0.16 0.13 1</ambient>"
             "<diffuse>0.90 0.45 0.38 1</diffuse>",
    "green": "<ambient>0.22 0.30 0.10 1</ambient>"
             "<diffuse>0.55 0.68 0.28 1</diffuse>",
}
FLOWER = ("<ambient>0.55 0.55 0.50 1</ambient>"
          "<diffuse>0.97 0.97 0.93 1</diffuse>")


def _vis(name: str, pose: str, geom: str, material: str) -> str:
    return (f'        <visual name="{name}"><pose>{pose}</pose>'
            f"<geometry>{geom}</geometry>"
            f"<material>{material}</material></visual>\n")


def procedural_plant(name: str, pose: str, z: float, seed: int) -> str:
    """One strawberry plant, from primitives. Deterministic in `seed`."""
    import math                                          # noqa: PLC0415
    import random                                        # noqa: PLC0415

    rng = random.Random(seed)
    v = [f'    <model name="{name}">\n      <static>true</static>\n'
         f"      <pose>{pose}</pose>\n      <link name=\"link\">\n"]

    # The crown: a squat dome the leaf stalks spring from.
    v.append(_vis("crown", f"0 0 {z + 0.02:.3f} 0 0 0",
                  "<sphere><radius>0.035</radius></sphere>", STEM_GREEN))

    # Seven leaf stalks, each ending in a trifoliate leaf. Splayed outward
    # and drooping, which is what gives the rosette its silhouette.
    for i in range(7):
        a = 2 * math.pi * i / 7 + rng.uniform(-0.25, 0.25)
        reach = rng.uniform(0.085, 0.125)
        lift = rng.uniform(0.055, 0.105)
        sx, sy = math.cos(a), math.sin(a)
        v.append(_vis(f"stalk{i}",
                      f"{sx * reach / 2:.3f} {sy * reach / 2:.3f} "
                      f"{z + lift / 2:.3f} 0 {math.atan2(reach, lift):.3f} "
                      f"{a:.3f}",
                      f"<cylinder><radius>0.004</radius>"
                      f"<length>{math.hypot(reach, lift):.3f}</length></cylinder>",
                      STEM_GREEN))
        # THE TRIFOLIATE LEAF: three leaflets, the middle one furthest out.
        # This is the shape that says "strawberry" and it is three flattened
        # ellipsoids -- the cheapest recognisable thing in the whole model.
        for k, (off, sc) in enumerate(((0.0, 1.0), (0.7, 0.82), (-0.7, 0.82))):
            la = a + off
            lr = reach + 0.045 * sc
            v.append(_vis(
                f"leaf{i}_{k}",
                f"{math.cos(la) * lr:.3f} {math.sin(la) * lr:.3f} "
                f"{z + lift:.3f} 0 0 {la:.3f}",
                f"<box><size>{0.075 * sc:.3f} {0.055 * sc:.3f} 0.006</size></box>",
                LEAF_GREEN))

    # Fruit trusses: arching out and DOWN, so the berries hang below the
    # canopy edge. A berry sitting on top of the leaves is the mistake every
    # stylised strawberry makes.
    stages = ["ripe", "ripe", "blush", "green", "green"]
    rng.shuffle(stages)
    for i, stage in enumerate(stages[:rng.randint(3, 5)]):
        a = 2 * math.pi * i / 5 + rng.uniform(-0.4, 0.4)
        reach = rng.uniform(0.10, 0.145)
        drop = rng.uniform(0.02, 0.06)
        bx, by = math.cos(a) * reach, math.sin(a) * reach
        v.append(_vis(f"truss{i}",
                      f"{bx / 2:.3f} {by / 2:.3f} {z + 0.03:.3f} 0 1.2 {a:.3f}",
                      f"<cylinder><radius>0.003</radius>"
                      f"<length>{reach:.3f}</length></cylinder>", STEM_GREEN))
        r = 0.019 if stage == "ripe" else (0.016 if stage == "blush" else 0.012)
        # Two stacked spheres, the lower one smaller: a strawberry is a cone
        # with a rounded shoulder, and one sphere reads as a cherry.
        v.append(_vis(f"berry{i}",
                      f"{bx:.3f} {by:.3f} {z - drop + r * 0.5:.3f} 0 0 0",
                      f"<sphere><radius>{r:.4f}</radius></sphere>",
                      FRUIT[stage]))
        v.append(_vis(f"berrytip{i}",
                      f"{bx:.3f} {by:.3f} {z - drop - r * 0.55:.3f} 0 0 0",
                      f"<sphere><radius>{r * 0.62:.4f}</radius></sphere>",
                      FRUIT[stage]))
        v.append(_vis(f"calyx{i}",
                      f"{bx:.3f} {by:.3f} {z - drop + r * 1.15:.3f} 0 0 0",
                      f"<box><size>{r * 2.4:.4f} {r * 2.4:.4f} 0.004</size></box>",
                      LEAF_GREEN))

    # A couple of white flowers, held above the leaves.
    for i in range(rng.randint(1, 3)):
        a = rng.uniform(0, 6.28)
        rr = rng.uniform(0.03, 0.075)
        v.append(_vis(f"flower{i}",
                      f"{math.cos(a) * rr:.3f} {math.sin(a) * rr:.3f} "
                      f"{z + rng.uniform(0.10, 0.14):.3f} 0 0 0",
                      "<sphere><radius>0.011</radius></sphere>", FLOWER))

    v.append(COLLISION.format(cz=z + 0.06) + "\n")
    v.append("      </link>\n    </model>\n")
    return "".join(v)


def swap_procedural(sdf: str, z: float, every_nth: int = 1) -> tuple[str, int]:
    """Replace plant models with the procedural strawberry, same contract."""
    out, replaced, seen, pos = [], 0, 0, 0
    pattern = re.compile(r'    <model name="(plant_\d+_\d+)">.*?</model>\n',
                         re.S)
    for m in pattern.finditer(sdf):
        out.append(sdf[pos:m.start()])
        pos = m.end()
        name = m.group(1)
        pose_m = re.search(r"<pose>([^<]+)</pose>", m.group(0))
        if pose_m is None or seen % max(1, every_nth):
            out.append(m.group(0))
            seen += 1
            continue
        seen += 1
        out.append(procedural_plant(name, pose_m.group(1).strip(), z,
                                    sum(ord(c) for c in name)))
        replaced += 1
    out.append(sdf[pos:])
    return "".join(out), replaced


def build_procedural(world: Path, z: float, every_nth: int) -> str:
    if not world.exists():
        raise FileNotFoundError(
            f"{world} does not exist. Generate it first:\n"
            "    python3 -m agri.world.make_world")
    sdf = world.read_text()
    if 'name="crown"' in sdf:
        raise ValueError(
            f"{world} already has procedural plants. Regenerate first:\n"
            "    python3 -m agri.world.make_world")
    swapped, n = swap_procedural(sdf, z, every_nth)
    if n == 0:
        raise ValueError(f"no plant models found in {world}")
    world.write_text(swapped)
    return (f"{n} procedural strawberry plant(s) written\n"
            f"  rosette of trifoliate leaves, fruit hanging on trusses "
            f"below the canopy, three ripeness stages\n"
            f"  crown at z = {z:.2f} m, collision left a primitive cylinder")


# =====================================================================
#  VARIETY: SEVERAL MESHES, ALTERNATED
# =====================================================================
#: STL carries geometry and NOTHING ELSE -- no colour, no material, no UV.
#: A strawberry mesh therefore renders in one flat colour, and the berries
#: disappear into the leaves. Stated here rather than discovered in Gazebo.
#:
#: Green is the honest choice: a plant that is mostly leaf, drawn in leaf
#: colour, is wrong about the fruit. Drawn in red it would be wrong about
#: everything, and would also give the floor camera's red-dominance
#: detector 24 large red objects to be confused by -- they are out of its
#: field of view (Sec. alignment), but building that coincidence into the
#: scene on purpose would be asking for it.
MESH_MATERIAL = ("<ambient>0.06 0.20 0.05 1</ambient>"
                 "<diffuse>0.16 0.44 0.13 1</diffuse>"
                 "<specular>0.10 0.16 0.08 1</specular>")


def mesh_plant_sdf(name: str, pose: str, uri: str, scale: float,
                   lift: float, base_z: float, yaw: float,
                   textured: bool = False, roll: float = 0.0) -> str:
    # A TEXTURED MESH MUST NOT BE GIVEN A MATERIAL. Ours would override the
    # one in the file and turn a photographed strawberry plant into a flat
    # green silhouette -- which is precisely the limitation the textured
    # asset exists to remove. Only an untextured mesh gets tinted.
    material = "" if textured else f"          <material>{MESH_MATERIAL}</material>"
    # SDF applies Rz(yaw) . Ry(pitch) . Rx(roll), so a +90 deg roll stands a
    # Y-up glTF upright FIRST and the yaw then spins it about the world's
    # vertical -- which is the order that gives a plant standing up and
    # turned, rather than one lying down and rolled.
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <visual name="plant">
          <pose>0 0 {base_z + lift:.4f} {roll:.4f} 0 {yaw:.4f}</pose>
          <geometry>
            <mesh>
              <uri>{uri}</uri>
              <scale>{scale:.5f} {scale:.5f} {scale:.5f}</scale>
            </mesh>
          </geometry>
{material}
        </visual>
{COLLISION.format(cz=base_z + 0.11)}
      </link>
    </model>
"""


def build_meshes(world: Path, meshes: list[Path], base_z: float,
                 every_nth: int, width_m: float = PLANT_WIDTH_M) -> str:
    """Stand one of several meshes on each plant position.

    Several rather than one, because 24 copies of a single asset read as 24
    copies. With two meshes and a per-plant yaw the row stops being a
    pattern -- which costs nothing at render time, since each distinct mesh
    is loaded once and instanced.
    """
    if not world.exists():
        raise FileNotFoundError(
            f"{world} does not exist. Generate it first:\n"
            "    python3 -m agri.world.make_world")
    if not meshes:
        raise ValueError("no mesh given")
    fitted = []
    for m in meshes:
        if not m.exists():
            raise FileNotFoundError(f"{m} does not exist")
        scale, lift, tris = fit(m, width_m)
        if m.suffix.lower() == ".glb" and not glb_is_self_contained(m):
            raise ValueError(
                f"{m.name} asks for a texture and does not carry the image.\n"
                "    Gazebo would load it and render it WHITE, with nothing\n"
                "    in any log to say why. Re-export with textures embedded\n"
                "    (Blender: glTF Binary, Images = Automatic), or use the\n"
                "    .stl and accept a flat colour.")
        fitted.append((m, scale, lift, tris))

    sdf = world.read_text()
    if "<mesh>" in sdf:
        raise ValueError(
            f"{world} already has mesh plants. Regenerate it first:\n"
            "    python3 -m agri.world.make_world")

    out, replaced, seen, pos = [], 0, 0, 0
    pattern = re.compile(r'    <model name="(plant_\d+_\d+)">.*?</model>\n',
                         re.S)
    for m in pattern.finditer(sdf):
        out.append(sdf[pos:m.start()])
        pos = m.end()
        name = m.group(1)
        pose_m = re.search(r"<pose>([^<]+)</pose>", m.group(0))
        if pose_m is None or seen % max(1, every_nth):
            out.append(m.group(0))
            seen += 1
            continue
        seen += 1
        # Deterministic in the plant's NAME, not random: regenerating twice
        # must give an empty diff, or every rebuild churns the world file.
        h = sum(ord(c) for c in name)
        path, scale, lift, _ = fitted[h % len(fitted)]
        yaw = (h * 37 % 360) * 3.14159265 / 180.0
        out.append(mesh_plant_sdf(name, pose_m.group(1).strip(),
                                  "file://" + str(path.resolve()),
                                  scale, lift, base_z, yaw,
                                  textured=path.suffix.lower() == ".glb"
                                  and glb_is_self_contained(path),
                                  roll=math.pi / 2 if needs_roll(path) else 0.0))
        replaced += 1
    out.append(sdf[pos:])
    world.write_text("".join(out))

    total = sum(t for _, _, _, t in fitted)
    lines = [f"{replaced} plant(s) from {len(fitted)} mesh(es), "
             f"alternated and rotated per plant"]
    flat = 0
    for path, scale, lift, tris in fitted:
        textured = (path.suffix.lower() == ".glb"
                    and glb_is_self_contained(path))
        flat += not textured
        lines.append(f"  {path.name:<20} {tris:>7,} tris  "
                     f"scale {scale:.4f}  lifted {lift * 100:.1f} cm  "
                     f"{fitted_height(path, width_m) * 100:.0f} cm tall  "
                     f"{'Y-up, rolled upright' if needs_roll(path) else 'Z-up'}"
                     f"  {'textured' if textured else 'flat colour'}")
    lines.append(f"  {width_m * 100:.0f} cm across, standing on z = "
                 f"{base_z:.2f} m; {replaced * total // max(1, len(fitted)):,}"
                 " triangles in the scene")
    lines.append("  collision stays a primitive cylinder whatever the mesh")
    if flat:
        lines.append(f"  {flat} mesh(es) carry no colour and render one flat "
                     "green -- that is the format, not a fault")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--world", type=Path,
                    default=Path(__file__).resolve().parents[2]
                    / "worlds" / "greenhouse_cloud.sdf")
    ap.add_argument("--mesh", type=Path,
                    default=Path(__file__).resolve().parents[2]
                    / "meshes" / "strawberry.dae",
                    help="the mesh file; .dae, .obj and .stl all work")
    ap.add_argument("--uri", default=None,
                    help="use this URI verbatim instead of --mesh, for a "
                         "model:// or package:// path")
    ap.add_argument("--scale", type=float, default=DEFAULT_SCALE)
    ap.add_argument("--z", type=float, default=DEFAULT_Z,
                    help="height of the mesh origin above the model origin; "
                         "the gutter top is 0.80 m")
    ap.add_argument("--every-nth", type=int, default=1, metavar="N",
                    help="place the mesh on every Nth plant only, and leave "
                         "the rest as blobs. The render budget, not a "
                         "preference: 24 detailed plants plus a GPU lidar "
                         "plus two cameras is where the frame rate goes.")
    ap.add_argument("--meshes", type=Path, nargs="+", default=None,
                    metavar="STL",
                    help="one or more meshes, alternated across the 24 "
                         "plants so the rows do not read as copies. Scale "
                         "and height are measured from each file.")
    ap.add_argument("--width", type=float, default=PLANT_WIDTH_M,
                    help="how wide a plant should be, in metres "
                         f"(default {PLANT_WIDTH_M})")
    ap.add_argument("--procedural", action="store_true",
                    help="build the plants from primitives instead of a "
                         "mesh. No asset, no licence, no texture files: a "
                         "rosette of trifoliate leaves with fruit hanging "
                         "on trusses. Not photoreal, and a long way past "
                         "the sphere blobs it replaces.")
    args = ap.parse_args(argv)
    try:
        if args.meshes:
            print(build_meshes(args.world, args.meshes, args.z,
                               args.every_nth, args.width))
        elif args.procedural:
            print(build_procedural(args.world, args.z, args.every_nth))
        else:
            print(build(args.world, args.mesh, args.scale, args.z,
                        args.every_nth, args.uri))
    except (FileNotFoundError, ValueError) as exc:
        print(f"make_plants: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
