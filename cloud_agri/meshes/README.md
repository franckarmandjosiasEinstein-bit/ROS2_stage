# meshes/

Three textured strawberry plants, alternated across the greenhouse so that
24 pots do not read as 24 copies of one asset. A fourth was supplied and
is rejected; see below.

| file | triangles | textured |
|---|---:|---|
| `strawberry_1.glb` | 7 557 | yes |
| `strawberry_3.glb` | 3 975 | yes |
| `strawberry_4.glb` | 7 277 | yes |
| `strawberry_a.stl` | 7 680 | no — kept as a fallback |
| `strawberry_b.stl` | 6 305 | no — kept as a fallback |
| `rejected/strawberry_baked_base_plate.glb` | 8 134 | yes, **and unusable** |

The glob is `strawberry_[0-9].glb`, so anything under `rejected/` stays out
of the greenhouse without needing a list to be maintained.

To put them in the world:

```bash
python3 -m agri.world.make_world      # the greenhouse + the 48 marks
python3 -m agri.world.make_plants --meshes meshes/strawberry_[0-9].glb
```

**The second command is not optional if you want plants that look like
plants**, and it is separate from the first for a reason: the mesh URI it
writes is an ABSOLUTE path, so a world generated on one machine cannot be
committed and used on another. `worlds/` is generated, never edited, and
never carries a mesh path from somebody else's disk.

**Textures are capped at 512 px.** The three meshes arrived carrying two
4096×4096 JPEGs each: 2 MB on disk, and **536 MB of GPU memory** once
uploaded — on a laptop whose integrated graphics take that out of system
RAM. Gazebo loaded so slowly that the launch declared the bridge broken,
and then the kernel killed it. The plant is 27 cm across and lands on about
320 pixels of the photograph; a 4096 texture is 150 times more data than
that, all of it averaged away by the mip chain before it reaches a screen.
Re-encoded at 512 they cost 8 MB in total and look identical.

`build_meshes` now refuses a mesh over the budget rather than warning about
it, because the failure it causes is an out-of-memory kill during load,
which says nothing about textures. To fix one:

```bash
python3 -m agri.world.textures meshes/whatever.glb          # rewrite it
python3 -m agri.world.textures --check meshes/*.glb         # just report
```

That is ~150 k triangles across the 24 plants. They are instanced — each
distinct file is uploaded to the GPU once — so the cost is four meshes, not
twenty-four. If the frame rate suffers on a laptop, drop to one variant
(`--meshes meshes/strawberry_3.glb`, the lightest at 3 975) or thin the
plants with `--every-nth 2`.

## What makes a mesh usable here

| | |
|---|---|
| format | `.glb` (binary glTF) — one file, textures inside. `.stl` also loads, with no colour. |
| up axis | either. `.glb`/`.gltf` are read as **Y up**, per the glTF specification, and rolled +90° in the SDF pose. `.stl`/`.dae` are read as Z up. |
| origin | anywhere: `plants.py` measures the bounding box and lifts the mesh so its lowest point rests on the gutter |
| size | anything: it is scaled to `PLANT_WIDTH_M` (27 cm) whatever units it came in |
| triangles | under ~15 k each. The 24 plants share the render budget with a GPU lidar and two cameras. |
| textures | **must be embedded.** See below. |

## The two traps, both of which cost a demonstration

### One: the mesh lies on its side, and you cannot see it

glTF fixes **+Y up** in its specification. `fit()` assumed Z for every
format, because the STLs came first and the GLBs were added without
re-asking. Every plant went into the greenhouse lying on its side.

A rosette lying down still looks like a plant from most angles, so this
survived a whole review. What gave it away was the one mesh with a base
plate baked in: the plate lies in the plant's XZ plane, so with Y treated
as horizontal it **stood up as a dark vertical wall** in the middle of
every photograph. The bug report was "why are the plates arranged like
that".

Gazebo does not rotate a glTF for you. `up_axis()` decides per format and
`mesh_plant_sdf()` writes the +90° roll into the pose — before the yaw, so
the plant stands up first and then spins about the world's vertical.

### Two: a texture that is referenced and not embedded

A glTF can reference a `baseColorTexture` and ship none of the image data.
Gazebo loads the mesh, finds no texture, and renders it **white** — no
error, nothing in any log. Three of the first four `.glb` files supplied did
exactly that; only one carried its JPEGs. The re-exported set in this
directory all carry theirs, and the pre-flight suite checks each one.

`glb_is_self_contained()` is the check, and `make_plants` refuses rather
than letting you discover it in the simulator. In Blender the setting is
**glTF Binary (.glb)** with **Images: Automatic**.

## A textured mesh is never repainted

`mesh_plant_sdf()` writes a `<material>` only for meshes that have no
colour of their own. Applying ours to a photographed plant would flatten it
to one green silhouette, which is the exact limitation the textured asset
exists to remove. The suite asserts both halves of that rule.

## What a baked base plate does, and why one mesh is in `rejected/`

`rejected/strawberry_baked_base_plate.glb` carries a display plinth — 20
triangles spanning 1.76 square units at the bottom of its Y axis — welded
into the **same primitive** as the plant. It is not a separate node, so it
cannot be dropped at load time; removing it means editing the mesh.

It also poisons `fit()`, which scales the mesh so its widest horizontal
dimension is 27 cm. With the plate included, the *plate* becomes 27 cm and
the plant shrinks to fit under it.

If you want it back, open it in Blender, select the plate's faces, delete
them, re-export as glTF Binary with Images: Automatic, and put it back as
`strawberry_2.glb`. The pre-flight suite checks the rejected one stays
rejected, so it will tell you if it drifts back in.

## Collision is never the mesh

Every plant gets a primitive collision cylinder; the mesh is visual only.
A mesh collision asks the physics engine to test every triangle on every
contact query, 24 times, every millisecond. The lidar still sees the
cylinder, which is what the brake reasons about.
