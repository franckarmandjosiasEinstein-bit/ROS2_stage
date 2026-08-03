#!/usr/bin/env python3
"""Run a model over a folder and write YOLO label files stage 8 can score.

This is the bridge between training and measurement, and it exists so that the
learned model and the colour-threshold baseline are scored by the SAME code on
the SAME images. Reporting a model's own validation mAP against a number
someone quoted for the old detector compares nothing.

Two modes:

    --weights best.pt        the learned model
    --baseline-colour        the CURRENT colour-threshold detector, reimplemented
                             here exactly as strawberry_detector applies it

The second is the point. Without it there is no honest comparison: the old
detector was measured in simulation, on different images, by a different
scorer. Run both over the held-out real test set and the difference means
something.

    python3 predict_to_labels.py --weights best.pt --images test/images --out preds/yolo
    python3 predict_to_labels.py --baseline-colour --images test/images --out preds/colour

    ros2 run youbot_commissioning stage8_detector -- \\
        --labels test/labels --predict preds/yolo --baseline preds/colour

WRITING AN EMPTY FILE IS MANDATORY
An image with no detection gets an EMPTY .txt, never no file. stage8 counts a
missing prediction file as "the model was not run here" and warns; an empty one
is a real, scoreable claim of "nothing in this image". Confusing the two moves
false negatives around silently.
"""

from __future__ import annotations

import argparse
import os
import sys

IMG_EXT = (".png", ".jpg", ".jpeg", ".bmp")


# --------------------------------------------------------------------------
#  The colour-threshold baseline, transcribed from
#  youbot_control/youbot_control/lib/vision.py so the comparison is against
#  the detector that is actually running on the robot, not a rough equivalent.
# --------------------------------------------------------------------------
def colour_boxes(rgb, value_min=60, red_ratio=0.33, min_pixels=6):
    """Ratio-based red segmentation + connected components -> boxes.

    This is the method being replaced. Its known weakness, and the reason the
    learned model exists, is that the ratio is invariant to illumination
    SCALE but not to illumination COLOUR: warm light lowers G/R across the
    whole scene and everything passes.
    """
    import numpy as np
    from scipy import ndimage            # optional; falls back below

    r = rgb[:, :, 0].astype(np.int16)
    g = rgb[:, :, 1].astype(np.int16)
    b = rgb[:, :, 2].astype(np.int16)
    limit = red_ratio * r
    mask = (r > value_min) & (r >= g) & (r >= b) & (g < limit) & (b < limit)

    lab, n = ndimage.label(mask)
    h, w = mask.shape
    out = []
    for i in range(1, n + 1):
        ys, xs = np.where(lab == i)
        if ys.size < min_pixels:
            continue
        x0, x1 = xs.min(), xs.max()
        y0, y1 = ys.min(), ys.max()
        out.append((0,
                    ((x0 + x1) / 2.0 + 0.5) / w,
                    ((y0 + y1) / 2.0 + 0.5) / h,
                    (x1 - x0 + 1) / w,
                    (y1 - y0 + 1) / h,
                    1.0))
    return out


def colour_boxes_noscipy(rgb, value_min=60, red_ratio=0.33, min_pixels=6):
    """Same, with an explicit flood fill, for machines without scipy."""
    import numpy as np

    r = rgb[:, :, 0].astype(np.int16)
    g = rgb[:, :, 1].astype(np.int16)
    b = rgb[:, :, 2].astype(np.int16)
    limit = red_ratio * r
    mask = (r > value_min) & (r >= g) & (r >= b) & (g < limit) & (b < limit)

    h, w = mask.shape
    seen = np.zeros_like(mask)
    out = []
    for y0 in range(h):
        for x0 in range(w):
            if not mask[y0, x0] or seen[y0, x0]:
                continue
            stack = [(y0, x0)]
            seen[y0, x0] = True
            xs, ys = [], []
            while stack:
                y, x = stack.pop()
                xs.append(x)
                ys.append(y)
                for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    ny, nx = y + dy, x + dx
                    if (0 <= ny < h and 0 <= nx < w
                            and mask[ny, nx] and not seen[ny, nx]):
                        seen[ny, nx] = True
                        stack.append((ny, nx))
            if len(xs) < min_pixels:
                continue
            out.append((0,
                        (min(xs) + max(xs) + 1) / 2.0 / w,
                        (min(ys) + max(ys) + 1) / 2.0 / h,
                        (max(xs) - min(xs) + 1) / w,
                        (max(ys) - min(ys) + 1) / h,
                        1.0))
    return out


def load_rgb(path):
    from PIL import Image
    import numpy as np
    return np.asarray(Image.open(path).convert("RGB"))


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--images", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--weights", default=None)
    ap.add_argument("--baseline-colour", action="store_true",
                    help="run the CURRENT colour threshold instead of a model")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--imgsz", type=int, default=960)
    ap.add_argument("--device", default="")
    args = ap.parse_args()

    if bool(args.weights) == bool(args.baseline_colour):
        print("give exactly one of --weights or --baseline-colour")
        return 2

    img_dir = os.path.expanduser(args.images)
    out_dir = os.path.expanduser(args.out)
    os.makedirs(out_dir, exist_ok=True)
    images = [f for f in sorted(os.listdir(img_dir))
              if f.lower().endswith(IMG_EXT)]
    if not images:
        print(f"no images under {img_dir}")
        return 2

    written = boxes = empty = 0

    if args.baseline_colour:
        try:
            from scipy import ndimage  # noqa: F401
            fn = colour_boxes
        except ImportError:
            print("scipy not found -- using the slower explicit flood fill")
            fn = colour_boxes_noscipy
        for name in images:
            rgb = load_rgb(os.path.join(img_dir, name))
            dets = fn(rgb)
            stem = os.path.splitext(name)[0]
            with open(os.path.join(out_dir, stem + ".txt"), "w") as fh:
                for cls, cx, cy, w, h, conf in dets:
                    fh.write(f"{cls} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f} "
                             f"{conf:.4f}\n")
            written += 1
            boxes += len(dets)
            empty += 1 if not dets else 0
    else:
        try:
            from ultralytics import YOLO
        except ImportError:
            print("ultralytics is not installed: pip install -r requirements.txt")
            return 2
        model = YOLO(os.path.expanduser(args.weights))
        for name in images:
            res = model.predict(os.path.join(img_dir, name),
                                imgsz=args.imgsz, conf=args.conf,
                                device=args.device or None, verbose=False)[0]
            stem = os.path.splitext(name)[0]
            n = 0
            with open(os.path.join(out_dir, stem + ".txt"), "w") as fh:
                for b in res.boxes:
                    cls = int(b.cls.item())
                    cx, cy, bw, bh = b.xywhn[0].tolist()
                    fh.write(f"{cls} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f} "
                             f"{float(b.conf.item()):.4f}\n")
                    n += 1
            written += 1
            boxes += n
            empty += 1 if n == 0 else 0

    print(f"wrote {written} label files to {out_dir}: {boxes} boxes, "
          f"{empty} images with no detection (empty files, not missing ones)")
    print("\nNow score BOTH against the same ground truth:")
    print("  ros2 run youbot_commissioning stage8_detector -- \\")
    print("      --labels <test/labels> --predict <yolo preds> "
          "--baseline <colour preds>")
    return 0


if __name__ == "__main__":
    sys.exit(main())
