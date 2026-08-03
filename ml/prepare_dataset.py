#!/usr/bin/env python3
"""Merge every data source into one YOLO tree with honest splits.

WHY MERGING NEEDS A SCRIPT AND NOT A `cp`

Three things go wrong when datasets are combined by hand, and all three are
invisible afterwards -- they show up only as a model that scores well and
fails in the field.

1. LEAKAGE BETWEEN SPLITS. Frames from one capture session are highly
   correlated: a robot at 0.2 m/s photographs the same plant twenty times.
   Splitting those at random puts near-duplicates in both train and val, the
   validation score climbs, and it measures nothing. This script splits BY
   SOURCE GROUP, never by individual image, so a session lands wholly in one
   split.

2. SYNTHETIC IMAGES IN THE TEST SET. A test set containing Gazebo renders
   cannot tell you whether the model works on a real greenhouse -- which is
   the only question. Sources are tagged `real` or `synthetic`, and by default
   the test split is drawn from REAL sources only. The script refuses to
   proceed without at least one real source unless you say so explicitly.

3. CLASS ID COLLISIONS. Every public dataset numbers its classes from 0. A
   strawberry set's class 0 and a glass set's class 0 mean different things,
   and concatenating them trains a model on a contradiction. Per-source class
   remapping is mandatory here, not optional.

USAGE
    python3 prepare_dataset.py \\
        --source ~/youbot_datasets/twin_morning:synthetic \\
        --source ~/downloads/strawdi_yolo:real:0=0 \\
        --source ~/photos_serre:real \\
        --out ~/strawberry_ds

A source is `PATH:KIND[:MAP]` where KIND is `real` or `synthetic` and MAP is
an optional comma-separated `old=new` class remap (unlisted classes are
dropped, loudly).
"""

from __future__ import annotations

import argparse
import os
import random
import shutil
import sys
from collections import Counter

import yaml

IMG_EXT = (".png", ".jpg", ".jpeg", ".bmp", ".ppm")


def load_classes(path: str) -> dict:
    with open(path) as fh:
        data = yaml.safe_load(fh)
    return {int(k): v for k, v in data["names"].items()}


def parse_source(spec: str):
    """`PATH:KIND[:MAP]` -> (path, kind, {old: new} or None)."""
    parts = spec.split(":")
    if len(parts) < 2:
        raise SystemExit(f"source {spec!r} needs PATH:KIND (real|synthetic)")
    path = os.path.expanduser(parts[0])
    kind = parts[1].lower()
    if kind not in ("real", "synthetic"):
        raise SystemExit(f"source {spec!r}: KIND must be real or synthetic")
    remap = None
    if len(parts) > 2 and parts[2].strip():
        remap = {}
        for pair in parts[2].split(","):
            old, new = pair.split("=")
            remap[int(old)] = int(new)
    return path, kind, remap


def find_pairs(root: str):
    """Every (image, label) pair under a YOLO-layout directory.

    Accepts both `<root>/images` + `<root>/labels` and a flat directory with
    the .txt beside the image, because the two conventions are equally common
    and getting it wrong yields an empty dataset with no error.
    """
    img_dir = os.path.join(root, "images")
    lab_dir = os.path.join(root, "labels")
    if not os.path.isdir(img_dir):
        img_dir = lab_dir = root
    pairs = []
    if not os.path.isdir(img_dir):
        return pairs
    for name in sorted(os.listdir(img_dir)):
        if not name.lower().endswith(IMG_EXT):
            continue
        stem = os.path.splitext(name)[0]
        lab = os.path.join(lab_dir, stem + ".txt")
        pairs.append((os.path.join(img_dir, name),
                      lab if os.path.exists(lab) else None))
    return pairs


def read_label(path, remap, dropped: Counter):
    """Read one YOLO label file, applying the class remap."""
    if path is None:
        return None                      # unlabelled: not the same as empty
    out = []
    with open(path) as fh:
        for line in fh:
            p = line.split()
            if len(p) < 5:
                continue
            cls = int(float(p[0]))
            if remap is not None:
                if cls not in remap:
                    dropped[cls] += 1
                    continue
                cls = remap[cls]
            out.append((cls, *(float(v) for v in p[1:5])))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", action="append", required=True,
                    help="PATH:KIND[:MAP], repeatable")
    ap.add_argument("--out", required=True)
    ap.add_argument("--classes", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "classes.yaml"))
    ap.add_argument("--val-frac", type=float, default=0.15)
    ap.add_argument("--test-frac", type=float, default=0.15)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--allow-synthetic-test", action="store_true",
                    help="let synthetic frames into the test split. Only for "
                         "a sanity run -- the resulting score does not "
                         "predict field behaviour.")
    args = ap.parse_args()

    names = load_classes(args.classes)
    out = os.path.expanduser(args.out)
    rng = random.Random(args.seed)

    sources = [parse_source(s) for s in args.source]
    real = [s for s in sources if s[1] == "real"]
    if not real and not args.allow_synthetic_test:
        print("REFUSING: every source is synthetic, so the test set would be "
              "renders. A score on rendered images does not predict a real "
              "greenhouse. Add a real source, or pass --allow-synthetic-test "
              "if you know that is what you want.")
        return 2

    # ---- gather, keeping each source as one indivisible group -----------
    groups = []            # (name, kind, [(img, label_rows)])
    dropped = Counter()
    for path, kind, remap in sources:
        pairs = find_pairs(path)
        if not pairs:
            print(f"WARNING: no images found under {path} -- skipping")
            continue
        items = [(img, read_label(lab, remap, dropped)) for img, lab in pairs]
        groups.append((os.path.basename(path.rstrip("/")) or path, kind, items))
        print(f"  {path}: {len(items)} images ({kind})")
    if not groups:
        print("nothing to do: no source contained images")
        return 2
    if dropped:
        print(f"  dropped labels with unmapped classes: {dict(dropped)}")

    # ---- split BY GROUP, test from real sources only --------------------
    real_groups = [g for g in groups if g[1] == "real"]
    syn_groups = [g for g in groups if g[1] == "synthetic"]
    rng.shuffle(real_groups)

    test_groups, val_groups, train_groups = [], [], []
    if args.allow_synthetic_test and not real_groups:
        rng.shuffle(syn_groups)
        pool, syn_groups = syn_groups, []
    else:
        pool = real_groups

    total_real = sum(len(g[2]) for g in pool)
    want_test = total_real * args.test_frac
    want_val = total_real * args.val_frac
    got_test = got_val = 0
    for g in pool:
        if got_test < want_test:
            test_groups.append(g)
            got_test += len(g[2])
        elif got_val < want_val:
            val_groups.append(g)
            got_val += len(g[2])
        else:
            train_groups.append(g)
    # Everything synthetic trains. It never validates and never tests.
    train_groups += syn_groups

    if not test_groups:
        print("WARNING: the test split is EMPTY. With only one real source "
              "there is nothing to hold out -- split it into sessions first.")

    # ---- write ----------------------------------------------------------
    counts = {}
    for split, gs in (("train", train_groups), ("val", val_groups),
                      ("test", test_groups)):
        idir = os.path.join(out, split, "images")
        ldir = os.path.join(out, split, "labels")
        os.makedirs(idir, exist_ok=True)
        os.makedirs(ldir, exist_ok=True)
        n_img = n_box = n_empty = n_unlabelled = 0
        per_class = Counter()
        for gname, _kind, items in gs:
            for img, rows in items:
                if rows is None:
                    n_unlabelled += 1
                    continue          # never ship an unlabelled image
                stem = f"{gname}_{os.path.splitext(os.path.basename(img))[0]}"
                shutil.copy2(img, os.path.join(
                    idir, stem + os.path.splitext(img)[1]))
                with open(os.path.join(ldir, stem + ".txt"), "w") as fh:
                    for cls, cx, cy, w, h in rows:
                        fh.write(f"{cls} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n")
                        per_class[cls] += 1
                    if not rows:
                        n_empty += 1
                n_img += 1
                n_box += len(rows)
        counts[split] = {"images": n_img, "boxes": n_box,
                         "empty": n_empty, "unlabelled_skipped": n_unlabelled,
                         "per_class": {names.get(c, c): n
                                       for c, n in sorted(per_class.items())}}

    data_yaml = {
        "path": out,
        "train": "train/images",
        "val": "val/images",
        "test": "test/images",
        "names": names,
    }
    with open(os.path.join(out, "data.yaml"), "w") as fh:
        yaml.safe_dump(data_yaml, fh, sort_keys=False, allow_unicode=True)
    with open(os.path.join(out, "splits.yaml"), "w") as fh:
        yaml.safe_dump({
            "counts": counts,
            "train_sources": [g[0] for g in train_groups],
            "val_sources": [g[0] for g in val_groups],
            "test_sources": [g[0] for g in test_groups],
            "split_rule": "by source group, never by image: frames from one "
                          "session are near-duplicates and splitting them at "
                          "random leaks the validation set",
            "test_rule": ("real sources only"
                          if not args.allow_synthetic_test
                          else "SYNTHETIC ALLOWED -- score does not predict "
                               "field behaviour"),
        }, fh, sort_keys=False, allow_unicode=True)

    print(f"\nwrote {out}/data.yaml")
    for split, c in counts.items():
        print(f"  {split:5s} {c['images']:5d} images, {c['boxes']:6d} boxes, "
              f"{c['empty']:4d} empty  {c['per_class']}")
    if counts["train"]["boxes"] and not counts["test"]["images"]:
        print("\nReminder: with no test split you can tune but you cannot "
              "report a number.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
