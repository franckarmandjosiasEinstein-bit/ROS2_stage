#!/usr/bin/env python3
"""import_real -- turn a downloaded real-image dataset into our YOLO layout.

WHY THIS EXISTS

The plan says "train on real strawberries", and the README lists where to get
them. A list of dataset names is not a path: every public set ships in a
different format and numbers its classes from 0 with its own meanings, so
"download StrawDI" is followed by a day of writing converters. This is that
day, done once.

Four input formats cover essentially everything you will find:

    strawdi   images + per-instance PNG masks   (StrawDI_Db1 and similar)
    yolo      images + .txt boxes + data.yaml   (Roboflow, most Kaggle sets)
    coco      images + one annotations.json     (COCO-style exports)
    voc       images + one .xml per image       (Pascal VOC style)

Output is always the same: <out>/images and <out>/labels in OUR class ids from
classes.yaml, ready for prepare_dataset.py as a `real` source.

START WITH --inspect. The first real problem with a downloaded dataset is
working out what it actually is; the archive rarely matches its description.

    python3 ml/import_real.py --inspect ~/downloads/StrawDI_Db1

THE CLASS MAPPING IS NOT OPTIONAL AND NOT GUESSABLE

A strawberry set's class 0 and a glass set's class 0 mean different things.
Worse, a strawberry set's "strawberry" may mean "ripe strawberry" or "any
strawberry", and those are different classes for us -- the second one poisons
the unripe class, which exists specifically as a hard negative. So the mapping
is stated explicitly on the command line, `--map` is required for every format
that carries class names, and unmapped classes are DROPPED loudly rather than
silently folded into class 0.

RIPENESS IN MASK DATASETS

StrawDI and most segmentation sets do not label ripeness at all: a mask is a
strawberry, full stop. Everything therefore lands in one class, and which one
is your decision, not the importer's. `--default-class` makes you say it, and
the tool warns that a set imported as all-ripe will teach the model that green
berries are ripe if the set contains any.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from collections import Counter

import yaml

IMG_EXT = (".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff")


# --------------------------------------------------------------------- utils
def load_our_classes(path: str) -> dict:
    with open(path) as fh:
        return {int(k): v for k, v in yaml.safe_load(fh)["names"].items()}


def parse_map(spec: str | None, ours: dict) -> dict | None:
    """`--map "strawberry=0,unripe=ripe_strawberry"` -> {source: our_id}."""
    if not spec:
        return None
    by_name = {v: k for k, v in ours.items()}
    out = {}
    for pair in spec.split(","):
        if "=" not in pair:
            raise SystemExit(f"--map entry {pair!r} needs the form src=dest")
        src, dest = (p.strip() for p in pair.split("=", 1))
        if dest.isdigit():
            dest_id = int(dest)
        elif dest in by_name:
            dest_id = by_name[dest]
        else:
            raise SystemExit(
                f"--map destination {dest!r} is not one of our classes: "
                f"{sorted(by_name)}")
        if dest_id not in ours:
            raise SystemExit(f"--map destination id {dest_id} is not in "
                             f"classes.yaml ({sorted(ours)})")
        out[src] = dest_id
    return out


def find_images(root: str) -> list[str]:
    hits = []
    for base, _dirs, files in os.walk(root):
        for f in files:
            if f.lower().endswith(IMG_EXT):
                hits.append(os.path.join(base, f))
    return sorted(hits)


def stem(path: str) -> str:
    return os.path.splitext(os.path.basename(path))[0]


def need_numpy():
    try:
        import numpy as np
        from PIL import Image
        return np, Image
    except ImportError:
        raise SystemExit("mask import needs pillow and numpy:  "
                         "pip install -r ml/requirements.txt")


class Writer:
    """Writes the output tree and keeps the tally."""

    def __init__(self, out: str, prefix: str):
        self.img_dir = os.path.join(out, "images")
        self.lab_dir = os.path.join(out, "labels")
        os.makedirs(self.img_dir, exist_ok=True)
        os.makedirs(self.lab_dir, exist_ok=True)
        self.prefix = prefix
        self.n_img = 0
        self.n_box = 0
        self.n_empty = 0
        self.per_class = Counter()
        self.dropped = Counter()

    def add(self, image_path: str, boxes) -> None:
        name = f"{self.prefix}_{stem(image_path)}"
        ext = os.path.splitext(image_path)[1]
        shutil.copy2(image_path, os.path.join(self.img_dir, name + ext))
        with open(os.path.join(self.lab_dir, name + ".txt"), "w") as fh:
            for cls, cx, cy, w, h in boxes:
                fh.write(f"{cls} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n")
                self.per_class[cls] += 1
        self.n_img += 1
        self.n_box += len(boxes)
        if not boxes:
            self.n_empty += 1


# ------------------------------------------------------------------ inspect
def inspect(src: str) -> int:
    print(f"inspecting {src}\n")
    if not os.path.isdir(src):
        print("  not a directory")
        return 2
    imgs = find_images(src)
    txts, xmls, jsons, yamls, masks = [], [], [], [], []
    for base, _d, files in os.walk(src):
        for f in files:
            p = os.path.join(base, f)
            low = f.lower()
            if low.endswith(".txt"):
                txts.append(p)
            elif low.endswith(".xml"):
                xmls.append(p)
            elif low.endswith(".json"):
                jsons.append(p)
            elif low.endswith((".yaml", ".yml")):
                yamls.append(p)
    # A "label"/"mask"/"gt" directory of images alongside an image directory
    # is the segmentation signature.
    for p in imgs:
        parts = {q.lower() for q in p.split(os.sep)}
        if parts & {"label", "labels", "mask", "masks", "gt", "annotations"}:
            masks.append(p)

    print(f"  images : {len(imgs)}")
    print(f"  of which in a label/mask directory : {len(masks)}")
    print(f"  .txt   : {len(txts)}")
    print(f"  .xml   : {len(xmls)}")
    print(f"  .json  : {len(jsons)}")
    print(f"  .yaml  : {len(yamls)}")

    for y in yamls[:3]:
        try:
            d = yaml.safe_load(open(y)) or {}
        except Exception:
            continue
        if "names" in d:
            print(f"\n  {os.path.relpath(y, src)} declares classes:")
            names = d["names"]
            items = (names.items() if isinstance(names, dict)
                     else enumerate(names))
            for k, v in items:
                print(f"    {k}: {v}")
            print("\n  -> looks like YOLO. Use:")
            print("     --from yolo --map \"" +
                  ",".join(f"{v}=<our class>" for _k, v in
                           (names.items() if isinstance(names, dict)
                            else enumerate(names))) + "\"")
            return 0

    if masks and len(masks) * 3 > len(imgs):
        print("\n  -> looks like per-instance MASKS (StrawDI style). Use:")
        print("     --from strawdi --default-class ripe_strawberry")
        return 0
    if jsons:
        print("\n  -> a .json is present; if it has 'annotations' and "
              "'categories' it is COCO. Use --from coco --map ...")
        return 0
    if xmls:
        print("\n  -> one .xml per image: Pascal VOC. Use --from voc --map ...")
        return 0
    if txts:
        print("\n  -> .txt beside images with no data.yaml: probably YOLO "
              "with undocumented class ids. Open one and see how many "
              "distinct first fields there are, then --from yolo --map ...")
        return 0
    print("\n  -> format not recognised. Look inside and tell the importer "
          "explicitly with --from.")
    return 1


# ------------------------------------------------------------------ strawdi
def import_strawdi(src, w: Writer, default_class: int, min_px: int) -> None:
    """images + per-instance PNG masks -> boxes.

    Instances are the distinct non-zero values in the mask. If there is only
    one non-zero value the mask is binary, and connected components are used
    instead -- otherwise every berry in the frame would merge into one box
    spanning the whole plant, which is worse than no label at all.
    """
    np, Image = need_numpy()
    pairs = []
    for base, _d, files in os.walk(src):
        low = os.path.basename(base).lower()
        if low not in ("img", "image", "images"):
            continue
        for cand in ("label", "labels", "mask", "masks", "gt"):
            lab_dir = os.path.join(os.path.dirname(base), cand)
            if os.path.isdir(lab_dir):
                for f in sorted(files):
                    if not f.lower().endswith(IMG_EXT):
                        continue
                    for ext in IMG_EXT:
                        m = os.path.join(lab_dir, stem(f) + ext)
                        if os.path.exists(m):
                            pairs.append((os.path.join(base, f), m))
                            break
                break
    if not pairs:
        raise SystemExit(
            "no image/mask pairs found. Expected an 'img' directory beside a "
            "'label' (or mask/gt) directory. Run --inspect first.")

    for img_path, mask_path in pairs:
        mask = np.asarray(Image.open(mask_path))
        if mask.ndim == 3:
            mask = mask[:, :, 0]
        H, W = mask.shape
        values = [v for v in np.unique(mask) if v != 0]
        boxes = []
        if len(values) > 1:
            regions = [(mask == v) for v in values]
        else:
            regions = list(_components(mask > 0, np))
        for reg in regions:
            ys, xs = np.where(reg)
            if xs.size < min_px:
                continue
            x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()
            boxes.append((default_class,
                          (x0 + x1 + 1) / 2.0 / W, (y0 + y1 + 1) / 2.0 / H,
                          (x1 - x0 + 1) / W, (y1 - y0 + 1) / H))
        w.add(img_path, boxes)


def _components(binary, np):
    """4-connectivity components, iterative. Only used for binary masks."""
    seen = np.zeros_like(binary)
    H, Wd = binary.shape
    for y0 in range(H):
        for x0 in range(Wd):
            if not binary[y0, x0] or seen[y0, x0]:
                continue
            stack = [(y0, x0)]
            seen[y0, x0] = True
            comp = np.zeros_like(binary)
            while stack:
                y, x = stack.pop()
                comp[y, x] = True
                for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    ny, nx = y + dy, x + dx
                    if (0 <= ny < H and 0 <= nx < Wd
                            and binary[ny, nx] and not seen[ny, nx]):
                        seen[ny, nx] = True
                        stack.append((ny, nx))
            yield comp


# --------------------------------------------------------------------- yolo
def import_yolo(src, w: Writer, mapping: dict, ours: dict) -> None:
    names = None
    for base, _d, files in os.walk(src):
        for f in files:
            if f.lower() in ("data.yaml", "data.yml"):
                d = yaml.safe_load(open(os.path.join(base, f))) or {}
                n = d.get("names")
                if isinstance(n, dict):
                    names = {int(k): v for k, v in n.items()}
                elif isinstance(n, list):
                    names = dict(enumerate(n))
    if names is None:
        raise SystemExit(
            "no data.yaml with class names found, so the source class ids are "
            "undocumented and cannot be mapped safely. Open a label file, work "
            "out what each id means, and pass --map \"0=ripe_strawberry,...\" "
            "using the NUMBERS as source keys.")
    print("  source classes:", names)

    for img in find_images(src):
        parts = img.split(os.sep)
        if "labels" in parts:
            continue                      # do not import label images as data
        lab = None
        for cand in (img.rsplit(".", 1)[0] + ".txt",
                     img.replace(os.sep + "images" + os.sep,
                                 os.sep + "labels" + os.sep
                                 ).rsplit(".", 1)[0] + ".txt"):
            if os.path.exists(cand):
                lab = cand
                break
        if lab is None:
            continue
        boxes = []
        for line in open(lab):
            p = line.split()
            if len(p) < 5:
                continue
            sid = int(float(p[0]))
            key = names.get(sid, str(sid))
            if key in mapping:
                dest = mapping[key]
            elif str(sid) in mapping:
                dest = mapping[str(sid)]
            else:
                w.dropped[key] += 1
                continue
            boxes.append((dest, *(float(v) for v in p[1:5])))
        w.add(img, boxes)


# --------------------------------------------------------------------- coco
def import_coco(src, w: Writer, mapping: dict) -> None:
    js = [os.path.join(b, f) for b, _d, fs in os.walk(src)
          for f in fs if f.lower().endswith(".json")]
    ann_file = None
    for j in js:
        try:
            d = json.load(open(j))
        except Exception:
            continue
        if isinstance(d, dict) and "annotations" in d and "images" in d:
            ann_file, data = j, d
            break
    if ann_file is None:
        raise SystemExit("no COCO json with 'images' and 'annotations' found")
    cats = {c["id"]: c["name"] for c in data.get("categories", [])}
    print("  source categories:", cats)
    imgs = {i["id"]: i for i in data["images"]}
    by_img: dict[int, list] = {}
    for a in data["annotations"]:
        by_img.setdefault(a["image_id"], []).append(a)

    root = os.path.dirname(ann_file)
    for iid, info in imgs.items():
        path = None
        for cand in (os.path.join(root, info["file_name"]),
                     os.path.join(root, "images", info["file_name"])):
            if os.path.exists(cand):
                path = cand
                break
        if path is None:
            continue
        W, H = float(info["width"]), float(info["height"])
        boxes = []
        for a in by_img.get(iid, []):
            key = cats.get(a["category_id"], str(a["category_id"]))
            if key not in mapping:
                w.dropped[key] += 1
                continue
            x, y, bw, bh = a["bbox"]           # COCO: top-left x, y, w, h
            boxes.append((mapping[key], (x + bw / 2) / W, (y + bh / 2) / H,
                          bw / W, bh / H))
        w.add(path, boxes)


# ---------------------------------------------------------------------- voc
def import_voc(src, w: Writer, mapping: dict) -> None:
    import xml.etree.ElementTree as ET
    for img in find_images(src):
        xml = img.rsplit(".", 1)[0] + ".xml"
        if not os.path.exists(xml):
            continue
        root = ET.parse(xml).getroot()
        size = root.find("size")
        W = float(size.find("width").text)
        H = float(size.find("height").text)
        boxes = []
        for obj in root.findall("object"):
            key = obj.find("name").text.strip()
            if key not in mapping:
                w.dropped[key] += 1
                continue
            b = obj.find("bndbox")
            x0, y0 = float(b.find("xmin").text), float(b.find("ymin").text)
            x1, y1 = float(b.find("xmax").text), float(b.find("ymax").text)
            boxes.append((mapping[key], (x0 + x1) / 2 / W, (y0 + y1) / 2 / H,
                          (x1 - x0) / W, (y1 - y0) / H))
        w.add(img, boxes)


# --------------------------------------------------------------------- main
def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--inspect", metavar="DIR",
                    help="report what a downloaded folder contains and exit")
    ap.add_argument("--src")
    ap.add_argument("--out")
    ap.add_argument("--from", dest="fmt",
                    choices=["strawdi", "yolo", "coco", "voc"])
    ap.add_argument("--map", help='e.g. "strawberry=ripe_strawberry,'
                                  'unripe=unripe_strawberry"')
    ap.add_argument("--default-class", default=None,
                    help="class for mask datasets that do not label ripeness")
    ap.add_argument("--prefix", default=None,
                    help="filename prefix; defaults to the source folder name")
    ap.add_argument("--min-mask-px", type=int, default=12)
    ap.add_argument("--classes", default=os.path.join(here, "classes.yaml"))
    args = ap.parse_args()

    if args.inspect:
        return inspect(os.path.expanduser(args.inspect))
    if not (args.src and args.out and args.fmt):
        ap.error("--src, --out and --from are required (or use --inspect)")

    ours = load_our_classes(args.classes)
    by_name = {v: k for k, v in ours.items()}
    src = os.path.expanduser(args.src)
    out = os.path.expanduser(args.out)
    prefix = args.prefix or os.path.basename(src.rstrip("/")) or "real"
    w = Writer(out, prefix)

    if args.fmt == "strawdi":
        if args.default_class is None:
            print("--default-class is required for mask datasets.\n"
                  "They do not label ripeness: a mask is a strawberry, full "
                  "stop. Which of our classes that becomes is your decision, "
                  f"not the importer's. Choose from {sorted(by_name)}.")
            return 2
        dc = (int(args.default_class) if args.default_class.isdigit()
              else by_name.get(args.default_class))
        if dc is None:
            print(f"unknown class {args.default_class!r}; "
                  f"choose from {sorted(by_name)}")
            return 2
        import_strawdi(src, w, dc, args.min_mask_px)
        if ours[dc] == "ripe_strawberry":
            print("\nWARNING: everything in this set was imported as "
                  "ripe_strawberry. If the images contain green berries, the "
                  "model is now being taught that green berries are ripe -- "
                  "which destroys the very distinction unripe_strawberry "
                  "exists to make. Look at a sample before training.")
    else:
        mapping = parse_map(args.map, ours)
        if mapping is None:
            print("--map is required for this format, and it is not "
                  "guessable: a set's 'strawberry' may mean 'ripe' or 'any "
                  "strawberry', and those are different classes for us. Run "
                  "--inspect to see the source class names.")
            return 2
        if args.fmt == "yolo":
            import_yolo(src, w, mapping, ours)
        elif args.fmt == "coco":
            import_coco(src, w, mapping)
        else:
            import_voc(src, w, mapping)

    print(f"\nwrote {out}")
    print(f"  {w.n_img} images, {w.n_box} boxes, {w.n_empty} with no object")
    print(f"  per class: "
          f"{ {ours.get(c, c): n for c, n in sorted(w.per_class.items())} }")
    if w.dropped:
        print(f"  DROPPED unmapped source classes: {dict(w.dropped)}")
        print("  Those objects are now absent from the labels. If any of them "
              "IS fruit, every one is a false negative the model will be "
              "punished for finding. Map them or remove those images.")
    if not w.n_img:
        print("  nothing imported -- run --inspect and check --from")
        return 1
    print(f"\nNext:  python3 ml/prepare_dataset.py --source {out}:real ...")
    return 0


if __name__ == "__main__":
    sys.exit(main())
