# meshes/

Drop the strawberry plant mesh here as `strawberry.dae`, then:

    python3 -m agri.world.make_world          # regenerate, blobs
    python3 -m agri.world.make_plants         # swap in the mesh

Nothing in this directory is generated, and nothing else in the project
depends on it: with no mesh here the world still builds, with the Phase B
sphere plants.

## What a usable mesh looks like

| | |
|---|---|
| format | `.dae` (Collada) preferred; `.obj`+`.mtl` and `.stl` also load |
| up axis | **Z up**. Most stores export Y-up; convert on export or pass a rotation. |
| origin | at the **root ball**, not at the canopy centre |
| size | a strawberry plant is 20–30 cm across, ~25 cm tall. Use `--scale` rather than editing the file. |
| triangles | **under ~15 k per plant** if you want all 24. See below. |
| textures | relative paths, and the image files beside the mesh |
| licence | must permit redistribution, or keep the file out of git and document where it came from |

## The render budget is the real constraint

24 plants x 50 000 triangles is 1.2 M triangles, in a scene that also runs
a GPU lidar, two rendering cameras and physics at 1 ms steps. That is where
the frame rate goes on a laptop with integrated graphics.

Three ways out, in order of preference:

1. Export a **decimated** version of the asset (most stores ship an LOD, or
   Blender's Decimate modifier at 0.2 keeps the silhouette).
2. `--every-nth 3` puts the mesh on eight plants and leaves sixteen as
   blobs. The rows still read as strawberries.
3. Accept a lower frame rate. Gazebo will run; it will not be pleasant.

## Collision is never the mesh

`plants.py` gives every plant a primitive collision cylinder and uses the
mesh for the visual only. A mesh collision would ask the physics engine to
test every triangle on every contact query. The lidar still sees the
cylinder, which is what the brake reasons about.
