# meshes/

`strawberry.glb` is the plant used by the greenhouse. Put it in place, then:

```bash
python3 -m agri.world.make_world      # the greenhouse + the 48 marks
python3 -m agri.world.make_plants --meshes meshes/strawberry.glb
```

**The second command is not optional if you want plants that look like
plants**, and it is separate from the first for a reason: the mesh URI it
writes is an ABSOLUTE path, so a world generated on one machine cannot be
committed and used on another. `worlds/` is generated, never edited, and
never carries a mesh path from somebody else's disk.

## What makes a mesh usable here

| | |
|---|---|
| format | `.glb` (binary glTF) — one file, textures inside. `.stl` also loads, with no colour. |
| up axis | **Z up** |
| origin | anywhere: `plants.py` measures the bounding box and lifts the mesh so its lowest point rests on the gutter |
| size | anything: it is scaled to `PLANT_WIDTH_M` (27 cm) whatever units it came in |
| triangles | under ~15 k. 24 plants share the render budget with a GPU lidar and two cameras. |
| textures | **must be embedded.** See below. |

## The trap that cost three of the four plants offered to this project

A glTF can reference a `baseColorTexture` and ship none of the image data.
Gazebo loads the mesh, finds no texture, and renders it **white** — no
error, nothing in any log. Three of the four `.glb` files supplied did
exactly that; only one carried its two JPEGs.

`glb_is_self_contained()` checks for it and `make_plants` refuses rather
than letting you discover it in the simulator. In Blender the setting is
**glTF Binary (.glb)** with **Images: Automatic**.

## Collision is never the mesh

Every plant gets a primitive collision cylinder; the mesh is visual only.
A mesh collision asks the physics engine to test every triangle on every
contact query, 24 times, every millisecond. The lidar still sees the
cylinder, which is what the brake reasons about.
