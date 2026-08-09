"""textures -- make a photogrammetry asset fit in a laptop's memory.

WHAT HAPPENED

The three strawberry meshes each carry two 4096x4096 JPEGs. Compressed
that is 2 MB a file and looks harmless. Uploaded to a GPU it is not: a
4096x4096 texture is 64 MiB of RGBA, 89 MiB with mipmaps, and six of them
is over half a gigabyte -- before a single triangle, and on a laptop whose
integrated graphics take that out of system RAM.

Gazebo spent so long loading the world that the launch's own watchdog
declared the bridge broken, and then the kernel killed the process.

WHAT SIZE IS ACTUALLY NEEDED

The plant is 27 cm across and is photographed from 64 cm through a lens
that frames 49 cm. It lands on about half the width of a 640x480 image:
roughly 320 pixels, at the one moment anybody looks at it closely. In the
Gazebo window it is smaller.

A 4096-pixel texture on a 320-pixel object is twelve times more texels
than pixels in each direction -- 150 times the data, all of it averaged
away by the mip chain before it reaches the screen. 512 is already
generous, and it is 64 times less memory.

This is not a compromise on how the plant looks. It is removing detail
that no pixel of the output was ever going to show.

WHY THE FILES ARE REWRITTEN AND NOT DOWNSCALED AT LOAD TIME

Because Gazebo has no such setting, and because a 2 MB file in git that
costs 500 MB of RAM to open is a trap for whoever clones this next. The
budget is checked by the pre-flight suite so a future 4K asset is refused
with the command that fixes it, rather than discovered by an OOM kill.
"""

from __future__ import annotations

import io
import json
import struct
from pathlib import Path

#: The longest edge a texture in this project may have.
#:
#: 512 against the ~320 pixels the plant ever occupies. Not 256: the
#: photograph is the deliverable, and a 256 texture on a 320-pixel subject
#: would be visibly soft in the one image the report shows.
MAX_TEXTURE_PX = 512

#: JPEG quality for the re-encode. 88 is above the point where artefacts
#: are visible on foliage, and well below the size where it matters.
JPEG_QUALITY = 88

_GLB_MAGIC = 0x46546C67
_JSON_CHUNK = 0x4E4F534A
_BIN_CHUNK = 0x004E4942


class TextureError(Exception):
    pass


def _read_glb(path: Path) -> tuple[dict, bytes]:
    d = path.read_bytes()
    magic, _version, _length = struct.unpack_from("<III", d, 0)
    if magic != _GLB_MAGIC:
        raise TextureError(f"{path} is not a binary glTF")
    off, js, bin_ = 12, None, b""
    while off < len(d):
        clen, ctype = struct.unpack_from("<II", d, off)
        chunk = d[off + 8: off + 8 + clen]
        if ctype == _JSON_CHUNK:
            js = json.loads(chunk)
        elif ctype == _BIN_CHUNK:
            bin_ = chunk
        off += 8 + clen + (-clen % 4)
    if js is None:
        raise TextureError(f"{path} has no JSON chunk")
    return js, bin_


def _write_glb(path: Path, js: dict, bin_: bytes) -> None:
    jb = json.dumps(js, separators=(",", ":")).encode()
    jb += b" " * (-len(jb) % 4)               # chunks are 4-byte aligned
    bb = bin_ + b"\0" * (-len(bin_) % 4)
    total = 12 + 8 + len(jb) + (8 + len(bb) if bb else 0)
    out = bytearray()
    out += struct.pack("<III", _GLB_MAGIC, 2, total)
    out += struct.pack("<II", len(jb), _JSON_CHUNK) + jb
    if bb:
        out += struct.pack("<II", len(bb), _BIN_CHUNK) + bb
    path.write_bytes(bytes(out))


def texture_sizes(path: Path) -> list[tuple[int, int]]:
    """(width, height) of every embedded image, largest first."""
    from PIL import Image                                  # noqa: PLC0415

    js, bin_ = _read_glb(path)
    out = []
    for im in js.get("images", []):
        if "bufferView" not in im:
            continue
        bv = js["bufferViews"][im["bufferView"]]
        raw = bin_[bv.get("byteOffset", 0):
                   bv.get("byteOffset", 0) + bv["byteLength"]]
        with Image.open(io.BytesIO(raw)) as img:
            out.append((img.width, img.height))
    return sorted(out, reverse=True)


def gpu_megabytes(path: Path) -> float:
    """Roughly what these textures cost once uploaded, with mipmaps.

    RGBA regardless of the source format -- a driver expands a JPEG to
    four channels -- and 4/3 for the mip chain. Approximate on purpose:
    the number is here to make half a gigabyte obvious, not to be exact.
    """
    return sum(w * h * 4 * 4 / 3 for w, h in texture_sizes(path)) / 1e6


def oversized(path: Path, limit: int = MAX_TEXTURE_PX) -> list[tuple[int, int]]:
    return [s for s in texture_sizes(path) if max(s) > limit]


def shrink(path: Path, limit: int = MAX_TEXTURE_PX,
           quality: int = JPEG_QUALITY) -> str:
    """Rewrite the file with every texture capped at `limit` pixels.

    The whole binary chunk is rebuilt rather than patched in place. Image
    data is the only thing that changes size, but every bufferView after a
    changed one moves, so the offsets have to be recomputed anyway --
    doing it for all of them, in order, is both simpler and the only
    version that cannot leave one accessor pointing at the wrong bytes.

    Alignment matters and is easy to get wrong: a bufferView an accessor
    reads from must start on a 4-byte boundary, and glTF's own validator
    is the only thing that would have told us otherwise.
    """
    from PIL import Image                                  # noqa: PLC0415

    js, bin_ = _read_glb(path)
    if not oversized(path, limit):
        return f"{path.name}: already within {limit} px"

    image_views = {im["bufferView"] for im in js.get("images", [])
                   if "bufferView" in im}
    before = gpu_megabytes(path)

    new_bin = bytearray()
    changed = []
    for i, bv in enumerate(js["bufferViews"]):
        start, ln = bv.get("byteOffset", 0), bv["byteLength"]
        data = bin_[start:start + ln]

        if i in image_views:
            with Image.open(io.BytesIO(data)) as img:
                w, h = img.size
                if max(w, h) > limit:
                    k = limit / max(w, h)
                    small = img.convert("RGB").resize(
                        (max(1, round(w * k)), max(1, round(h * k))),
                        Image.LANCZOS)
                    buf = io.BytesIO()
                    small.save(buf, "JPEG", quality=quality, optimize=True)
                    data = buf.getvalue()
                    changed.append((w, h, small.width, small.height))

        pad = -len(new_bin) % 4
        new_bin += b"\0" * pad
        bv["byteOffset"] = len(new_bin)
        bv["byteLength"] = len(data)
        new_bin += data

    js["buffers"][0]["byteLength"] = len(new_bin)
    js["buffers"][0].pop("uri", None)
    _write_glb(path, js, bytes(new_bin))

    after = gpu_megabytes(path)
    lines = [f"{path.name}: {before:.0f} -> {after:.0f} MB of texture memory"]
    for w, h, nw, nh in changed:
        lines.append(f"    {w}x{h} -> {nw}x{nh}")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    import argparse                                        # noqa: PLC0415

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("meshes", nargs="+", type=Path)
    ap.add_argument("--limit", type=int, default=MAX_TEXTURE_PX)
    ap.add_argument("--check", action="store_true",
                    help="report and change nothing; exit 1 if any is over")
    args = ap.parse_args(argv)

    over = False
    for m in args.meshes:
        if not m.exists():
            print(f"{m} does not exist")
            return 1
        if args.check:
            big = oversized(m, args.limit)
            print(f"{m.name}: {gpu_megabytes(m):.0f} MB of texture memory, "
                  f"{[f'{w}x{h}' for w, h in texture_sizes(m)]}")
            if big:
                over = True
                print(f"    OVER the {args.limit} px budget")
        else:
            print(shrink(m, args.limit))
    return 1 if over else 0


if __name__ == "__main__":
    raise SystemExit(main())
