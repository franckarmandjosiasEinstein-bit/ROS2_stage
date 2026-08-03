#!/usr/bin/env python3
"""Train the strawberry / plant / glass detector.

Thin on purpose: Ultralytics does the work, and this file exists to pin the
handful of settings that actually matter for THIS problem and to say why, so
that nobody re-tunes them from a blog post six months from now.

THE TWO SETTINGS THAT MATTER MORE THAN THE MODEL SIZE

1. IMAGE SIZE. Strawberries are small objects. A 34 mm berry at 2 m subtends
   about 15 px in a 640 px frame with this camera's 1.2 rad field of view, and
   YOLO's coarsest head has stride 32 -- below roughly 20 px, recall falls off
   a cliff. Training at 960 px moves recall at range more than going from
   yolov8n to yolov8m does, and it is the first thing to raise if distant
   fruit is being missed.

2. HUE AUGMENTATION. Ultralytics defaults to hsv_h = 0.015, which is a very
   narrow band of colour shift. That default is wrong for this project
   specifically, because the exact failure being fixed IS a colour shift: the
   colour-threshold detector survives shade (illumination scale) and fails at
   17 h (illumination colour temperature). Training with the default hue range
   on midday images reproduces that failure in a neural network -- a more
   expensive way to get the same bug. hsv_h is raised to 0.05 here.

WHAT THIS SCRIPT DELIBERATELY DOES NOT DO

It does not report a final number. Validation mAP is for choosing a
checkpoint. The number that goes in the report comes from stage 8, on the held
-out real test set, against the colour-threshold baseline on identical images:

    python3 predict_to_labels.py --weights <best.pt> --images <test/images> --out <preds>
    ros2 run youbot_commissioning stage8_detector -- \\
        --labels <test/labels> --predict <preds> --baseline <colour_preds>
"""

from __future__ import annotations

import argparse
import os
import sys

import yaml


# Chosen for this problem, with the reasoning attached. Anything not listed
# here is an Ultralytics default and was left alone on purpose.
TUNED = {
    # --- the colour-robustness block -----------------------------------
    "hsv_h": 0.05,    # 3.3x the default. See the header: the failure mode
                      # being fixed is a hue shift, so the training data must
                      # contain hue shifts.
    "hsv_s": 0.7,     # saturation: wet fruit and overcast light
    "hsv_v": 0.5,     # brightness: sunlit vs shaded row, same aisle

    # --- the small-object block ----------------------------------------
    "mosaic": 1.0,    # four images per sample: the single most effective
                      # augmentation for small objects, because it puts them
                      # at more scales and positions than the data contains
    "close_mosaic": 10,   # off for the last 10 epochs: mosaic distorts object
                          # statistics, and the final epochs should see real
                          # framing
    "scale": 0.5,     # +/-50 % zoom: stands in for the 0.3-2.5 m range
    "copy_paste": 0.0,  # needs masks; enable if you annotate plants as masks

    # --- geometry -------------------------------------------------------
    "fliplr": 0.5,    # the robot drives an aisle in both directions
    "flipud": 0.0,    # it never drives upside down; this would be noise
    "degrees": 5.0,   # small: the camera is on a levelled base
    "perspective": 0.0,

    # --- optimisation ---------------------------------------------------
    "optimizer": "auto",
    "patience": 30,   # early stop; small agricultural sets plateau early
    "cos_lr": True,
}


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", required=True, help="data.yaml from prepare_dataset.py")
    ap.add_argument("--model", default="yolov8s",
                    help="yolov8n|s|m|l, or a .pt to fine-tune from")
    ap.add_argument("--imgsz", type=int, default=960,
                    help="960 or 1280. Do NOT drop to 640 for this problem.")
    ap.add_argument("--epochs", type=int, default=150)
    ap.add_argument("--batch", type=int, default=8,
                    help="-1 lets Ultralytics pick from free VRAM")
    ap.add_argument("--device", default="",
                    help="'0' for the first GPU, 'cpu' to suffer")
    ap.add_argument("--name", default="strawberry")
    ap.add_argument("--freeze", type=int, default=0,
                    help="freeze the first N layers. Use ~10 when "
                         "fine-tuning a synthetic-trained model on a few "
                         "hundred real images, so the backbone is not "
                         "destroyed by a small set.")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the resolved configuration and exit")
    args = ap.parse_args()

    data_path = os.path.expanduser(args.data)
    if not os.path.exists(data_path):
        print(f"no data.yaml at {data_path} -- run prepare_dataset.py first")
        return 2
    with open(data_path) as fh:
        data = yaml.safe_load(fh)

    cfg = dict(TUNED)
    cfg.update({
        "data": data_path,
        "imgsz": args.imgsz,
        "epochs": args.epochs,
        "batch": args.batch,
        "name": args.name,
        "freeze": args.freeze or None,
    })
    if args.device:
        cfg["device"] = args.device

    print("classes:", data.get("names"))
    print("resolved training configuration:")
    for k in sorted(cfg):
        print(f"  {k:14s} {cfg[k]}")
    if args.imgsz < 960:
        print("\nWARNING: imgsz < 960. A berry at 2 m is ~15 px at 640, below "
              "the stride-32 head's useful size. Expect recall at range to "
              "collapse and do not blame the model size for it.")
    if args.dry_run:
        return 0

    try:
        from ultralytics import YOLO
    except ImportError:
        print("\nultralytics is not installed:  pip install -r requirements.txt")
        return 2

    model = YOLO(args.model if args.model.endswith(".pt")
                 else f"{args.model}.pt")
    model.train(**cfg)

    print("\nTraining finished. The validation mAP above is for PICKING a "
          "checkpoint, not for reporting.")
    print("The number that goes in the report comes from stage 8, on the "
          "held-out REAL test split, against the colour baseline:")
    print("  python3 predict_to_labels.py --weights runs/detect/"
          f"{args.name}/weights/best.pt --images <test/images> --out <preds>")
    print("  ros2 run youbot_commissioning stage8_detector -- "
          "--labels <test/labels> --predict <preds> --baseline <colour_preds>")
    return 0


if __name__ == "__main__":
    sys.exit(main())
