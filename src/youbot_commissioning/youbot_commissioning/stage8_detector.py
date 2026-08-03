"""Stage 8 -- evaluate the fruit detector OFFLINE, against annotations.

Offline and honest. No robot, no simulator, no ROS graph required: a folder of
images, a folder of ground-truth labels, a folder of predictions, and
arithmetic. That is the point. A detector evaluated online, on the robot,
mixes its own errors with the pose error, the timing and the mission logic,
and the result cannot be attributed to anything.

WHAT IS MEASURED
    precision = TP / (TP + FP)   of what it claimed, how much was real
    recall    = TP / (TP + FN)   of what was real, how much it found
    F1        = harmonic mean
    per-class breakdown, because "ripe" and "unripe" fail differently

Matching is by IoU (default 0.5), greedy from the highest-confidence
prediction, each ground-truth box matched at most once -- the standard
protocol, stated so the number can be reproduced.

THE BASELINE THAT MUST BE BEATEN
The current colour-threshold detector measured, in simulation, roughly 69%
recall with about two of every three map estimates spurious. Any learned model
must be compared against that same colour threshold ON THE SAME REAL IMAGES,
or the comparison proves nothing. This script takes two prediction folders for
exactly that reason.

USAGE
    ros2 run youbot_commissioning stage8_detector -- \\
        --labels  ~/dataset/labels \\
        --predict ~/runs/yolov8n/labels \\
        --baseline ~/runs/colour_threshold/labels

Label format is YOLO: one .txt per image, lines of
    class cx cy w h [confidence]
with all coordinates normalised to [0, 1]. Predictions carry confidence,
ground truth does not.
"""

from __future__ import annotations

import argparse
import glob
import os
import sys

from youbot_commissioning.lib.report import Report


def load_boxes(path: str, with_conf: bool):
    """Read one YOLO label file -> list of (cls, cx, cy, w, h, conf)."""
    out = []
    if not os.path.exists(path):
        return out
    with open(path) as fh:
        for line in fh:
            parts = line.split()
            if len(parts) < 5:
                continue
            try:
                cls = int(float(parts[0]))
                cx, cy, w, h = (float(p) for p in parts[1:5])
                conf = float(parts[5]) if with_conf and len(parts) > 5 else 1.0
            except ValueError:
                continue
            out.append((cls, cx, cy, w, h, conf))
    return out


def iou(a, b) -> float:
    ax0, ay0 = a[1] - a[3] / 2.0, a[2] - a[4] / 2.0
    ax1, ay1 = a[1] + a[3] / 2.0, a[2] + a[4] / 2.0
    bx0, by0 = b[1] - b[3] / 2.0, b[2] - b[4] / 2.0
    bx1, by1 = b[1] + b[3] / 2.0, b[2] + b[4] / 2.0
    ix0, iy0 = max(ax0, bx0), max(ay0, by0)
    ix1, iy1 = min(ax1, bx1), min(ay1, by1)
    iw, ih = max(0.0, ix1 - ix0), max(0.0, iy1 - iy0)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    union = a[3] * a[4] + b[3] * b[4] - inter
    return inter / union if union > 0.0 else 0.0


def evaluate(label_dir: str, pred_dir: str, iou_thr: float, conf_thr: float):
    """Greedy IoU matching over every label file. Returns a metrics dict."""
    tp = fp = fn = 0
    per_class: dict[int, dict] = {}
    images = 0
    missing_pred = 0

    for lab_path in sorted(glob.glob(os.path.join(label_dir, "*.txt"))):
        images += 1
        stem = os.path.basename(lab_path)
        pred_path = os.path.join(pred_dir, stem)
        if not os.path.exists(pred_path):
            missing_pred += 1

        truth = load_boxes(lab_path, with_conf=False)
        preds = [p for p in load_boxes(pred_path, with_conf=True)
                 if p[5] >= conf_thr]
        preds.sort(key=lambda p: -p[5])

        used = [False] * len(truth)
        for p in preds:
            best_i, best_iou = -1, 0.0
            for i, t in enumerate(truth):
                if used[i] or t[0] != p[0]:
                    continue
                v = iou(p, t)
                if v > best_iou:
                    best_i, best_iou = i, v
            slot = per_class.setdefault(p[0], {"tp": 0, "fp": 0, "fn": 0})
            if best_i >= 0 and best_iou >= iou_thr:
                used[best_i] = True
                tp += 1
                slot["tp"] += 1
            else:
                fp += 1
                slot["fp"] += 1
        for i, t in enumerate(truth):
            if not used[i]:
                fn += 1
                per_class.setdefault(t[0], {"tp": 0, "fp": 0, "fn": 0})["fn"] += 1

    def prf(t, f, n):
        p = t / (t + f) if (t + f) else 0.0
        r = t / (t + n) if (t + n) else 0.0
        f1 = 2 * p * r / (p + r) if (p + r) else 0.0
        return p, r, f1

    precision, recall, f1 = prf(tp, fp, fn)
    for cls, s in per_class.items():
        s["precision"], s["recall"], s["f1"] = prf(s["tp"], s["fp"], s["fn"])
    return {
        "images": images,
        "missing_prediction_files": missing_pred,
        "tp": tp, "fp": fp, "fn": fn,
        "precision": precision, "recall": recall, "f1": f1,
        "per_class": per_class,
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Stage 8 -- offline fruit-detector evaluation")
    ap.add_argument("--labels", required=True,
                    help="ground-truth YOLO label directory")
    ap.add_argument("--predict", required=True,
                    help="prediction label directory (the model under test)")
    ap.add_argument("--baseline", default=None,
                    help="prediction directory for the colour-threshold "
                         "baseline, evaluated on the SAME images")
    ap.add_argument("--iou", type=float, default=0.5)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--min-precision", type=float, default=0.85)
    ap.add_argument("--min-recall", type=float, default=0.80)
    ap.add_argument("--min-images", type=int, default=100)
    args = ap.parse_args(
        [a for a in (argv if argv is not None else sys.argv[1:]) if a != "--"])

    report = Report(8, "detector", "Offline fruit-detector evaluation",
                    platform_name="offline")

    if not os.path.isdir(args.labels):
        print(f"label directory not found: {args.labels}")
        return 2

    model = evaluate(args.labels, args.predict, args.iou, args.conf)
    report.record("model", model)
    report.record("iou_threshold", args.iou)
    report.record("confidence_threshold", args.conf)
    print(f"model    : images={model['images']} TP={model['tp']} "
          f"FP={model['fp']} FN={model['fn']}  "
          f"P={model['precision']:.3f} R={model['recall']:.3f} "
          f"F1={model['f1']:.3f}")

    baseline = None
    if args.baseline:
        baseline = evaluate(args.labels, args.baseline, args.iou, args.conf)
        report.record("baseline", baseline)
        print(f"baseline : images={baseline['images']} TP={baseline['tp']} "
              f"FP={baseline['fp']} FN={baseline['fn']}  "
              f"P={baseline['precision']:.3f} R={baseline['recall']:.3f} "
              f"F1={baseline['f1']:.3f}")

    for cls, s in sorted(model["per_class"].items()):
        print(f"  class {cls}: P={s['precision']:.3f} R={s['recall']:.3f} "
              f"F1={s['f1']:.3f}  (TP {s['tp']} FP {s['fp']} FN {s['fn']})")

    report.check("annotated images", model["images"], ">=", args.min_images)
    report.check("missing prediction files",
                 model["missing_prediction_files"], "==", 0,
                 note="a missing file silently becomes 'detected nothing' and "
                      "inflates the false-negative count")
    report.check("precision", model["precision"], ">=", args.min_precision, "",
                 "the current colour threshold produces roughly two spurious "
                 "estimates out of three; precision is the metric that fixes "
                 "the wasted alignment time")
    report.check("recall", model["recall"], ">=", args.min_recall)
    if baseline is not None:
        report.check("beats the colour-threshold baseline on F1",
                     model["f1"] - baseline["f1"], ">", 0.0, "",
                     f"model {model['f1']:.3f} vs baseline "
                     f"{baseline['f1']:.3f} on identical images")

    report.finish()
    return 0 if report.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
