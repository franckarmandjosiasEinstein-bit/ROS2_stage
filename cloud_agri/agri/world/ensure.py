"""ensure -- the world and the robot exist, are current, and have plants.

WHY THIS FILE EXISTS

`worlds/greenhouse_cloud.sdf` and `urdf/youbot_agri.urdf` are generated.
They were also committed, and the README told you to regenerate them. Those
two facts cannot both be right, and the way they failed is worth writing
down because it cost a demonstration:

    git pull
    error: Your local changes to the following files would be overwritten
    by merge:  cloud_agri/worlds/greenhouse_cloud.sdf
    Aborting

The pull aborted. The next three lines of a pasted block ran anyway --
each line of a paste executes independently, so `&&` protects nothing past
the first newline -- and one of them regenerated the world WITHOUT plants.
Gazebo opened, showed green spheres, and nothing anywhere said that the
repository was four commits behind and the plant meshes had never been
downloaded. The only visible symptom was "the strawberries look wrong".

So: the generated files are no longer tracked, and this module makes sure
they are there and current before the simulator starts. `run_sim.sh` calls
it, which means the answer to "did I remember to run make_plants" is now
"you cannot forget".

WHAT COUNTS AS OUT OF DATE

Four things, and each one is a real failure that has happened:

    missing          a fresh clone has no world at all
    older than its   the catalogue moved the crosses and the world still
      generator      has the old ones -- the robot then parks correctly on
                     marks that are not where the paint is
    no plants        the world was built by make_world alone, so every
                     plant is a sphere
    dead mesh URI    the world names /home/someone-else/... because a mesh
                     path is ABSOLUTE. Gazebo renders nothing and says
                     nothing.

The last one is why this is a module rather than four lines of bash: it
needs to read the file and stat the paths inside it, and it needs to be
testable without a simulator.
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

WORLD = ROOT / "worlds" / "greenhouse_cloud.sdf"
URDF = ROOT / "urdf" / "youbot_agri.urdf"

#: The plant meshes, in the order build_meshes should alternate them.
#: A glob rather than a list so that dropping a fifth variant in works.
MESH_GLOB = "strawberry_[0-9].glb"

#: Touching any of these invalidates the world. Not every file in agri/ --
#: only the ones whose output ends up in the SDF. A list that included, say,
#: the Cloud would regenerate the greenhouse every time a dashboard colour
#: changed, and a regeneration nobody asked for is how a demonstration
#: discovers a broken generator thirty seconds before it starts.
WORLD_SOURCES = ("agri/world/make_world.py", "agri/world/plants.py",
                 "agri/catalogue.py", "agri/aisles.py", "agri/labels.py")

URDF_SOURCES = ("agri/world/make_robot.py", "agri/catalogue.py")


def meshes(root: Path = ROOT) -> list[Path]:
    return sorted((root / "meshes").glob(MESH_GLOB))


def _newest(paths) -> float:
    return max((p.stat().st_mtime for p in paths if p.exists()), default=0.0)


def dead_mesh_uris(world: Path) -> list[str]:
    """Absolute mesh paths named by this world that are not on this disk.

    THE ONE THAT IS INVISIBLE IN GAZEBO. A `file:///home/someone/...` URI
    from another machine loads nothing and logs nothing; the plant is
    simply absent from the render, which looks like a plant that was never
    added rather than a path that is wrong.
    """
    if not world.exists():
        return []
    out = []
    for uri in re.findall(r"<uri>file://([^<]+)</uri>", world.read_text()):
        if not Path(uri).exists():
            out.append(uri)
    return out


def why_stale(root: Path = ROOT) -> list[str]:
    """Every reason the generated assets need rebuilding. Empty means fine.

    Every reason, not the first: a world that is both stale and unplanted
    should say both, because an operator who fixes one and re-runs should
    not then be told about the other.
    """
    world = root / "worlds" / "greenhouse_cloud.sdf"
    urdf = root / "urdf" / "youbot_agri.urdf"
    reasons = []

    if not world.exists():
        reasons.append(f"{world.name} does not exist")
    else:
        newest = _newest([root / s for s in WORLD_SOURCES])
        if newest > world.stat().st_mtime:
            reasons.append(f"{world.name} is older than the code that "
                           "generates it (a pull moved the crosses?)")
        text = world.read_text()
        have = meshes(root)
        if have and "<mesh>" not in text:
            reasons.append(f"{world.name} has no plant meshes -- every plant "
                           "would render as a green sphere")
        dead = dead_mesh_uris(world)
        if dead:
            reasons.append(
                f"{world.name} names {len(dead)} mesh path(s) that do not "
                f"exist on this machine, starting with {dead[0]} -- Gazebo "
                "would render nothing there and say nothing about it")

    if not urdf.exists():
        reasons.append(f"{urdf.name} does not exist")
    elif _newest([root / s for s in URDF_SOURCES]) > urdf.stat().st_mtime:
        reasons.append(f"{urdf.name} is older than the code that generates it")

    return reasons


def _run(module: str, *args: str) -> str:
    """Run one generator in a child process, or raise with its output.

    A child rather than an import: the generators print a report worth
    keeping, they are entry points somebody may also run by hand, and a
    failure in one must not leave this process holding half-built state.
    """
    r = subprocess.run([sys.executable, "-m", module, *args],
                       capture_output=True, text=True, cwd=str(ROOT))
    if r.returncode != 0:
        raise RuntimeError(
            f"{module} failed ({r.returncode}):\n{r.stdout}{r.stderr}")
    return r.stdout.strip()


def ensure(root: Path = ROOT, force: bool = False) -> list[str]:
    """Build whatever is missing or stale. Returns what it did, or [].

    The world and the plants are ONE operation. make_plants refuses to swap
    meshes into a world that already has them -- correctly, since the swap
    is not idempotent -- so the only safe way to re-plant is to build the
    world again first. Doing that here rather than leaving it to the
    operator is the whole point of the module.
    """
    reasons = ["forced"] if force else why_stale(root)
    if not reasons:
        return []

    done = []
    world = root / "worlds" / "greenhouse_cloud.sdf"
    urdf = root / "urdf" / "youbot_agri.urdf"

    world_reasons = [r for r in reasons
                     if r.startswith(world.name) or r == "forced"]
    if world_reasons:
        world.parent.mkdir(parents=True, exist_ok=True)
        _run("agri.world.make_world")
        done.append(f"regenerated {world.name}")
        have = meshes(root)
        if have:
            report = _run("agri.world.make_plants", "--meshes",
                          *[str(m) for m in have])
            done.append(report.splitlines()[0]
                        if report else f"planted {len(have)} mesh(es)")
        else:
            done.append("no strawberry_N.glb in meshes/ -- plants stay "
                        "spheres. This is a `git pull` short of finished.")

    urdf_reasons = [r for r in reasons
                    if r.startswith(urdf.name) or r == "forced"]
    if urdf_reasons:
        urdf.parent.mkdir(parents=True, exist_ok=True)
        _run("agri.world.make_robot")
        done.append(f"regenerated {urdf.name}")

    return done


def main(argv: list[str] | None = None) -> int:
    import argparse                                       # noqa: PLC0415

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--force", action="store_true",
                    help="rebuild even if nothing looks stale")
    ap.add_argument("--check", action="store_true",
                    help="say what is stale and change nothing; exit 1 if "
                         "anything is")
    args = ap.parse_args(argv)

    if args.check:
        reasons = why_stale()
        for r in reasons:
            print(f"stale: {r}")
        if not reasons:
            print("the generated world and robot are current")
        return 1 if reasons else 0

    try:
        done = ensure(force=args.force)
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 1
    for line in done:
        print(line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
