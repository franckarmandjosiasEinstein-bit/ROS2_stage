# meshes/

Four textured strawberry plants, alternated across the greenhouse so that
24 pots do not read as 24 copies of one asset:

| file | triangles | textured |
|---|---:|---|
| `strawberry_1.glb` | 7 557 | yes |
| `strawberry_2.glb` | 8 134 | yes |
| `strawberry_3.glb` | 3 975 | yes |
| `strawberry_4.glb` | 7 277 | yes |
| `strawberry_a.stl` | 7 680 | no — kept as a fallback |
| `strawberry_b.stl` | 6 305 | no — kept as a fallback |

To put them in the world:

```bash
python3 -m agri.world.make_world      # the greenhouse + the 48 marks
python3 -m agri.world.make_plants --meshes meshes/strawberry_[1-4].glb
```

**The second command is not optional if you want plants that look like
plants**, and it is separate from the first for a reason: the mesh URI it
writes is an ABSOLUTE path, so a world generated on one machine cannot be
committed and used on another. `worlds/` is generated, never edited, and
never carries a mesh path from somebody else's disk.

That is ~162 k triangles across the 24 plants. They are instanced — each
distinct file is uploaded to the GPU once — so the cost is four meshes, not
twenty-four. If the frame rate suffers on a laptop, drop to one variant
(`--meshes meshes/strawberry_3.glb`, the lightest at 3 975) or thin the
plants with `--every-nth 2`.

## What makes a mesh usable here

| | |
|---|---|
| format | `.glb` (binary glTF) — one file, textures inside. `.stl` also loads, with no colour. |
| up axis | **Z up** |
| origin | anywhere: `plants.py` measures the bounding box and lifts the mesh so its lowest point rests on the gutter |
| size | anything: it is scaled to `PLANT_WIDTH_M` (27 cm) whatever units it came in |
| triangles | under ~15 k each. The 24 plants share the render budget with a GPU lidar and two cameras. |
| textures | **must be embedded.** See below. |

## The trap that cost the first four files offered to this project

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

## Collision is never the mesh

Every plant gets a primitive collision cylinder; the mesh is visual only.
A mesh collision asks the physics engine to test every triangle on every
contact query, 24 times, every millisecond. The lidar still sees the
cylinder, which is what the brake reasons about.
