#!/usr/bin/env python3
"""Widen the daylight range of a dataset, offline.

WHY A SEPARATE SCRIPT WHEN YOLO ALREADY AUGMENTS

Ultralytics' hsv_h shifts hue by a fraction of the colour wheel, uniformly and
symmetrically. That is a generic augmentation. What breaks this project is not
generic: it is the specific, one-directional shift produced by sunlight through
glass as the day ends. Light goes WARM -- the blue channel falls, the red rises
-- and every surface in the frame moves the same way at once. That is a
blackbody shift, not a hue rotation, and the colour-threshold detector fails
precisely because its G/R test cannot tell a warm scene from a red object.

So the training set gets copies of itself under simulated colour temperatures
from cold overcast to low evening sun. Labels are copied unchanged -- geometry
does not move -- which is the whole reason this can be done offline for free.

This is the augmentation that stands between a model trained on midday images
and a model that still works at 17 h. If you only ever capture the twin at one
lighting setting, run this.

    python3 augment_daylight.py --dataset ~/strawberry_ds --split train

Do NOT run it on val or test. Augmenting the test set measures the
augmentation, not the model.
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys

IMG_EXT = (".png", ".jpg", ".jpeg", ".bmp")

# Approximate linear RGB gains for a range of daylight colour temperatures,
# normalised so that ~6500 K (overcast noon) is neutral. Derived from the
# standard blackbody approximation; exact enough for augmentation, and the
# point is the DIRECTION and spread, not radiometric accuracy.
TEMPERATURES = {
    "overcast_7500K":  (0.92, 0.97, 1.10),
    "noon_6500K":      (1.00, 1.00, 1.00),
    "afternoon_5000K": (1.10, 1.00, 0.85),
    "golden_4000K":    (1.22, 0.98, 0.70),
    "sunset_3000K":    (1.38, 0.93, 0.55),
}


def apply_temperature(rgb, gains, exposure=1.0):
    """Multiply the channels, in a rough linear space, then clip.

    sRGB is gamma-encoded, so scaling it directly exaggerates the shift in the
    shadows. Decoding with gamma 2.2, scaling, and re-encoding keeps the shift
    where a real illuminant change puts it -- across the whole tone range
    rather than concentrated in the darks.
    """
    import numpy as np
    lin = (rgb.astype(np.float32) / 255.0) ** 2.2
    for c in range(3):
        lin[:, :, c] *= gains[c] * exposure
    out = np.clip(lin, 0.0, 1.0) ** (1.0 / 2.2)
    return (out * 255.0 + 0.5).astype(np.uint8)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dataset", required=True, help="the prepare_dataset.py tree")
    ap.add_argument("--split", default="train", choices=["train"],
                    help="train only, deliberately: augmenting val or test "
                         "measures the augmentation instead of the model")
    ap.add_argument("--temperatures", default="afternoon_5000K,golden_4000K,sunset_3000K",
                    help="comma-separated keys from TEMPERATURES")
    ap.add_argument("--exposure-jitter", type=float, default=0.15,
                    help="+/- fraction of brightness variation per copy")
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    try:
        import numpy as np
        from PIL import Image
    except ImportError:
        print("needs pillow and numpy:  pip install -r requirements.txt")
        return 2

    rng = np.random.default_rng(args.seed)
    root = os.path.expanduser(args.dataset)
    idir = os.path.join(root, args.split, "images")
    ldir = os.path.join(root, args.split, "labels")
    if not os.path.isdir(idir):
        print(f"no {idir} -- run prepare_dataset.py first")
        return 2

    wanted = [t.strip() for t in args.temperatures.split(",") if t.strip()]
    for t in wanted:
        if t not in TEMPERATURES:
            print(f"unknown temperature {t!r}. Known: {list(TEMPERATURES)}")
            return 2

    originals = [f for f in sorted(os.listdir(idir))
                 if f.lower().endswith(IMG_EXT) and "_ct" not in f]
    if not originals:
        print(f"no source images in {idir}")
        return 2

    made = 0
    for name in originals:
        stem, ext = os.path.splitext(name)
        lab = os.path.join(ldir, stem + ".txt")
        if not os.path.exists(lab):
            continue                     # never fabricate a label
        rgb = np.asarray(Image.open(os.path.join(idir, name)).convert("RGB"))
        for key in wanted:
            exposure = 1.0 + float(rng.uniform(-args.exposure_jitter,
                                               args.exposure_jitter))
            out = apply_temperature(rgb, TEMPERATURES[key], exposure)
            new_stem = f"{stem}_ct{key}"
            Image.fromarray(out).save(os.path.join(idir, new_stem + ext))
            # Labels are copied verbatim: an illuminant change moves no pixel.
            shutil.copy2(lab, os.path.join(ldir, new_stem + ".txt"))
            made += 1

    print(f"{len(originals)} source images -> {made} colour-temperature copies "
          f"in {args.split}")
    print("Labels were copied unchanged: changing the illuminant moves no "
          "geometry, which is why this is free.")
    print("\nSanity check before training: open a few *_ctsunset_3000K images. "
          "If the plants now look red to YOU, that is exactly the condition "
          "the colour threshold fails in, and now the model has seen it.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
