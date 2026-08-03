#!/usr/bin/env python3
"""Build the training handover PDF from the live ml/ sources.

Generated from the repository rather than written by hand, so the code in the
document is by construction the code that is in the tree. Re-run it after any
change to ml/ and hand over the new PDF.

    python3 ml/make_handover_pdf.py
"""

from __future__ import annotations

import os
import sys
from datetime import date

from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.platypus import (BaseDocTemplate, Frame, PageBreak, PageTemplate,
                                Paragraph, Preformatted, Spacer, Table,
                                TableStyle)

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "training_handover.pdf")

CODE_WIDTH = 96          # characters per line at 7 pt Courier on A4 margins
ACCENT = colors.HexColor("#2E5F9E")
WARN = colors.HexColor("#B03A2E")
OKC = colors.HexColor("#1E7B34")
CODEBG = colors.HexColor("#F4F4F7")


# --------------------------------------------------------------------------
def latin1(text: str) -> str:
    """Replace anything the built-in fonts cannot draw.

    reportlab's standard fonts are Latin-1. A stray arrow or dash renders as a
    black box, which in a handover document reads as a corrupted file.
    """
    repl = {
        "→": "->", "←": "<-", "↔": "<->", "⇒": "=>",
        "≥": ">=", "≤": "<=", "≠": "!=", "±": "+/-",
        "–": "-", "—": "--", "‘": "'", "’": "'",
        "“": '"', "”": '"', "…": "...", "×": "x",
        "•": "-", "°": " deg", "σ": "sigma", "δ": "delta",
        "∑": "sum", "√": "sqrt", "≈": "~", " ": " ",
        "✅": "[ok]", "❌": "[X]", "⚠": "[!]",
    }
    for k, v in repl.items():
        text = text.replace(k, v)
    return text.encode("latin-1", "replace").decode("latin-1")


def wrap_code(text: str, width: int = CODE_WIDTH):
    """Hard-wrap long code lines; Preformatted does not wrap on its own."""
    out = []
    for line in text.expandtabs(4).splitlines():
        line = latin1(line)
        if len(line) <= width:
            out.append(line)
            continue
        indent = len(line) - len(line.lstrip())
        pad = " " * min(indent + 4, width - 20)
        while len(line) > width:
            cut = line.rfind(" ", 0, width)
            if cut <= indent:
                cut = width
            out.append(line[:cut])
            line = pad + line[cut:].lstrip()
        out.append(line)
    return "\n".join(out)


# --------------------------------------------------------------------------
styles = getSampleStyleSheet()
S = {
    "title": ParagraphStyle("t", parent=styles["Title"], fontSize=19,
                            leading=23, textColor=ACCENT, spaceAfter=2),
    "sub": ParagraphStyle("s", parent=styles["Normal"], fontSize=10.5,
                          leading=14, textColor=colors.HexColor("#444444"),
                          alignment=TA_LEFT, spaceAfter=10),
    "h1": ParagraphStyle("h1", parent=styles["Heading1"], fontSize=14,
                         leading=17, textColor=ACCENT, spaceBefore=14,
                         spaceAfter=6),
    "h2": ParagraphStyle("h2", parent=styles["Heading2"], fontSize=11.5,
                         leading=14, textColor=colors.HexColor("#333333"),
                         spaceBefore=9, spaceAfter=4),
    "body": ParagraphStyle("b", parent=styles["Normal"], fontSize=9.4,
                           leading=13, spaceAfter=5),
    "bullet": ParagraphStyle("bu", parent=styles["Normal"], fontSize=9.4,
                             leading=13, leftIndent=12, bulletIndent=3,
                             spaceAfter=3),
    "warn": ParagraphStyle("w", parent=styles["Normal"], fontSize=9.6,
                           leading=13, textColor=WARN, borderPadding=6,
                           backColor=colors.HexColor("#FBE6E4"),
                           borderColor=WARN, borderWidth=0.8, spaceBefore=6,
                           spaceAfter=8),
    "note": ParagraphStyle("n", parent=styles["Normal"], fontSize=9.2,
                           leading=12.5, backColor=colors.HexColor("#EEF3FB"),
                           borderColor=ACCENT, borderWidth=0.6,
                           borderPadding=6, spaceBefore=5, spaceAfter=8),
    "code": ParagraphStyle("c", parent=styles["Code"], fontName="Courier",
                           fontSize=7.0, leading=8.4,
                           backColor=CODEBG, borderPadding=4,
                           spaceBefore=3, spaceAfter=7),
    "shell": ParagraphStyle("sh", parent=styles["Code"], fontName="Courier",
                            fontSize=7.8, leading=9.6,
                            backColor=colors.HexColor("#F0F4F0"),
                            borderColor=colors.HexColor("#CBD8CB"),
                            borderWidth=0.6, borderPadding=5,
                            spaceBefore=3, spaceAfter=8),
    "file": ParagraphStyle("f", parent=styles["Normal"], fontName="Courier-Bold",
                           fontSize=9.5, leading=12, textColor=ACCENT,
                           spaceBefore=12, spaceAfter=2),
}


def P(text, style="body"):
    return Paragraph(latin1(text), S[style])


def BUL(items):
    return [Paragraph(latin1("&bull;&nbsp; " + t), S["bullet"]) for t in items]


def TBL(rows, widths, header=True, small=False):
    fs = 7.8 if small else 8.4
    data = [[Paragraph(latin1(str(c)),
                       ParagraphStyle("cell", parent=S["body"], fontSize=fs,
                                      leading=fs + 2.6,
                                      spaceAfter=0,
                                      fontName="Helvetica-Bold" if (header and i == 0)
                                      else "Helvetica"))
             for c in row] for i, row in enumerate(rows)]
    t = Table(data, colWidths=widths, repeatRows=1 if header else 0)
    style = [
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#BBBBBB")),
        ("LEFTPADDING", (0, 0), (-1, -1), 4),
        ("RIGHTPADDING", (0, 0), (-1, -1), 4),
        ("TOPPADDING", (0, 0), (-1, -1), 3),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
    ]
    if header:
        style += [("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#DCE9F7"))]
    t.setStyle(TableStyle(style))
    return t


def listing(relpath: str, story: list) -> None:
    path = os.path.join(HERE, relpath)
    if not os.path.exists(path):
        story.append(P(f"[missing: {relpath}]", "warn"))
        return
    with open(path) as fh:
        text = fh.read()
    n = text.count("\n") + 1
    story.append(Paragraph(latin1(f"ml/{relpath}"), S["file"]))
    story.append(P(f"{n} lines", "sub"))
    story.append(Preformatted(wrap_code(text), S["code"]))


# --------------------------------------------------------------------------
def header_footer(canvas, doc):
    canvas.saveState()
    canvas.setFont("Helvetica", 7.2)
    canvas.setFillColor(colors.HexColor("#777777"))
    canvas.drawString(18 * mm, A4[1] - 11 * mm,
                      "Strawberry / plant / glass detector "
                      "- training handover")
    canvas.drawRightString(A4[0] - 18 * mm, A4[1] - 11 * mm,
                           "PFA Smart Agriculture - YouBot digital twin")
    canvas.setStrokeColor(colors.HexColor("#CCCCCC"))
    canvas.setLineWidth(0.4)
    canvas.line(18 * mm, A4[1] - 13 * mm, A4[0] - 18 * mm, A4[1] - 13 * mm)
    canvas.line(18 * mm, 13 * mm, A4[0] - 18 * mm, 13 * mm)
    canvas.drawString(18 * mm, 9 * mm, latin1(f"Generated {date.today().isoformat()} "
                                              "from the ml/ sources"))
    canvas.drawRightString(A4[0] - 18 * mm, 9 * mm, f"page {doc.page}")
    canvas.restoreState()


def build() -> str:
    doc = BaseDocTemplate(OUT, pagesize=A4,
                          leftMargin=18 * mm, rightMargin=18 * mm,
                          topMargin=18 * mm, bottomMargin=17 * mm,
                          title="Strawberry detector - training handover",
                          author="Franck Armand Josias Einstein")
    frame = Frame(doc.leftMargin, doc.bottomMargin,
                  doc.width, doc.height, id="main")
    doc.addPageTemplates([PageTemplate(id="all", frames=[frame],
                                       onPage=header_footer)])
    s: list = []

    # ---------------------------------------------------------- cover
    s.append(P("Training a detector for strawberries, plants and glass", "title"))
    s.append(P("Handover document. Everything needed to train the perception "
               "model for the autonomous greenhouse harvester: what the "
               "classes are, what data to collect, where to get it, the "
               "complete pipeline source, and how the result will be judged.",
               "sub"))

    s.append(P("Why this exists", "h1"))
    s.append(P("The detector currently running on the robot thresholds redness "
               "by channel ratio. Measured against the 127 known berry "
               "positions in the simulator it reaches <b>69% recall</b>, and "
               "<b>66 of its 111</b> map estimates match no real berry. The "
               "harvest phase spends <b>39% of its time</b> aligning on things "
               "that are not fruit."))
    s.append(P("It cannot be tuned out of that. Its test is a ratio, "
               "G &lt; 0.33R and B &lt; 0.33R, which is invariant to "
               "illumination <i>scale</i> - that is why it survives shade - "
               "but not to illumination <i>colour</i>. Late-afternoon light "
               "through glass lowers G/R for the entire scene, and at that "
               "point every surface passes the fruit test. A learned model "
               "fixes the class of problem rather than the instance, but only "
               "if it is trained on data that contains the failure."))

    # ---------------------------------------------------------- 1 classes
    s.append(P("1. The four classes", "h1"))
    s.append(TBL([
        ["Class", "id", "Annotation", "What it is for", "Priority"],
        ["ripe_strawberry", "0", "bounding box",
         "The harvest target. The only class the mission acts on. Small "
         "objects: a berry at 2 m is about 15 px in a 640 px frame.", "1"],
        ["unripe_strawberry", "1", "bounding box",
         "Not a target - a HARD NEGATIVE. Ripeness is a continuum, not a "
         "switch. A model trained only on ripe fruit learns 'roundish object "
         "on a plant' and reports every green berry. Its own class forces the "
         "boundary to be about colour maturity instead of shape.", "2"],
        ["strawberry_plant", "2", "box, or mask if affordable",
         "Locates the growing row, and constrains where a berry may "
         "plausibly be: a 'berry' detected over bare floor is wrong. Easy to "
         "detect, hard to box tightly - foliage has no clean boundary.", "3"],
        ["glass", "3", "SEGMENTATION mask",
         "The sim-to-real risk: a 2D lidar at glancing incidence on a pane "
         "returns nothing, so the wall is invisible. Read the warning in "
         "section 4 before investing here.", "4"],
    ], [30 * mm, 8 * mm, 26 * mm, 90 * mm, 14 * mm]))
    s.append(Spacer(1, 4))
    s.append(P("Class ids are fixed in <b>ml/classes.yaml</b> and are read by "
               "every script and by the robot-side scorer. Append new classes "
               "at the end; <b>never reorder</b>, or every label file ever "
               "written is silently remapped.", "note"))

    # ---------------------------------------------------------- 2 data
    s.append(P("2. What kind of data", "h1"))
    s.append(P("2.1 Format", "h2"))
    s.append(P("YOLO throughout: one <font face='Courier'>.txt</font> per "
               "image, lines of <font face='Courier'>class cx cy w h</font>, "
               "all coordinates normalised to [0, 1]."))
    s.append(P("An <b>empty file</b> means 'nothing here, and I am sure'. A "
               "<b>missing file</b> means 'unlabelled'. They are not the same, "
               "and confusing them inflates the false-negative count silently. "
               "Images with no fruit must get empty label files, not no file.",
               "note"))

    s.append(P("2.2 How much, and of what", "h2"))
    s.append(P("Volume matters less than coverage of the conditions that break "
               "the current detector. 500 images spanning a day beat 5000 "
               "taken in one hour."))
    s.append(TBL([
        ["Condition to cover", "Why it matters", "Rough share"],
        ["Morning, midday, late afternoon, overcast",
         "The colour-temperature failure. This is the single most important "
         "axis.", "4 blocks, balanced"],
        ["Both aisle sides (sunlit row, shaded row)",
         "Exposure extremes within one pass.", "50 / 50"],
        ["Ripe, half-ripe and green fruit",
         "Ripeness is a continuum; the boundary cases are where the model "
         "will be wrong.", "all three present"],
        ["Fruit occluded by leaves",
         "The majority case in reality. A model trained on clean fruit "
         "reports clean fruit only.", "at least 40% of berries"],
        ["Red objects that are NOT fruit: pipes, tools, crates, clothing",
         "This is where false positives come from. They must be in the "
         "training set to be rejected.", "at least 15% of frames"],
        ["Frames with no fruit at all",
         "Teaches 'nothing here'. Kept as empty label files.", "about 30% of frames"],
        ["Wet fruit, specular highlights",
         "A white glare splits one berry into two blobs.", "some"],
        ["Range 0.3 m to 2.5 m", "Small-object recall at range.", "spread evenly"],
    ], [52 * mm, 82 * mm, 34 * mm]))
    s.append(Spacer(1, 4))
    s.append(P("<b>First target: 2000 synthetic frames + 300 to 500 real "
               "annotated frames.</b> The real ones matter far more per image; "
               "the synthetic ones stop the model overfitting to the few real "
               "backgrounds available."))

    # ---------------------------------------------------------- 3 sources
    s.append(PageBreak())
    s.append(P("3. Where to get the data", "h1"))

    s.append(P("3.1 The digital twin - free, perfectly annotated, available tonight",
               "h2"))
    s.append(P("The simulated greenhouse knows where all 127 berries are, so "
               "it writes its own labels. This is the one thing a twin gives "
               "that a photographer does not: not pictures, but pictures with "
               "perfect ground truth, thousands of them, overnight."))
    s.append(Preformatted(wrap_code(
        "ros2 launch youbot_gazebo gazebo.launch.py\n"
        "ros2 run youbot_slam dataset_capture --ros-args \\\n"
        "    -p session:=twin_morning -p target_frames:=2000\n"
        "\n"
        "# record the conditions of each session - half the value of the set\n"
        "ros2 topic pub -1 /dataset/conditions std_msgs/String \\\n"
        "  \"{data: 'warm light, sun low from the west, north row shaded'}\""),
        S["shell"]))
    s.append(P("Output lands in <font face='Courier'>~/youbot_datasets/"
               "&lt;session&gt;/</font> as <font face='Courier'>images/</font>, "
               "<font face='Courier'>labels/</font>, "
               "<font face='Courier'>index.csv</font> and "
               "<font face='Courier'>dataset.json</font>, already in YOLO "
               "format."))
    s.append(P("<b>Labels never come from the colour detector.</b> They are "
               "projected from the ground-truth berry catalogue through "
               "<font face='Courier'>youbot_slam.lib.berry_view</font>, the "
               "same visibility model the scorer uses. Annotating with the "
               "detector would teach its replacement to reproduce its "
               "two-in-three false positives exactly.", "note"))
    s.append(P("<b>Limitation, stated plainly:</b> a model trained only on "
               "rendered images transfers imperfectly. Synthetic data stops "
               "overfitting; it does not replace real images. Plan for "
               "fine-tuning."))

    s.append(P("3.2 Public datasets - real images, today, no site access", "h2"))
    s.append(P("Names and descriptions below come from published work. "
               "<b>Verify the current URL and the licence before use</b> - "
               "hosting and terms change, and several are research-only."))
    s.append(TBL([
        ["Dataset", "What it is", "Notes"],
        ["StrawDI_Db1",
         "About 3100 real strawberry images from fields in Huelva, Spain, "
         "with instance segmentation masks.",
         "Best single starting point. Masks convert to boxes trivially. "
         "Research use."],
        ["Roboflow Universe (search 'strawberry ripeness', "
         "'strawberry detection')",
         "Dozens of community sets, many already split into ripe / unripe.",
         "Exports directly in YOLO format. Quality varies a lot - inspect "
         "before trusting. Check each licence."],
        ["DeepFruits (Sa et al., Sensors, 2016)",
         "Seven fruit types including strawberry, bounding boxes.",
         "Small but well documented."],
        ["Kaggle (search 'strawberry detection')",
         "Several sets, mixed quality.", "Same caveat as Roboflow."],
        ["GDD - Mei et al., 'Don't Hit Me! Glass Detection in Real-world "
         "Scenes', CVPR 2020",
         "About 3900 images with glass region masks.",
         "The reference dataset for glass."],
        ["Trans10K - Xie et al., ECCV 2020",
         "About 10400 images of transparent objects, two categories.",
         "Larger, broader than glass alone."],
    ], [42 * mm, 62 * mm, 64 * mm], small=True))
    s.append(Spacer(1, 4))
    s.append(P("<b>For plants:</b> the strawberry field datasets above already "
               "contain plants - annotate the plant class on those same images "
               "rather than hunting for a separate source. CVPPP has leaf "
               "segmentation data if more is wanted; PlantVillage is "
               "disease-focused and shot on plain backgrounds, which is poor "
               "context for this problem."))

    s.append(P("3.3 Real photographs of the actual greenhouse - the ones that "
               "matter most", "h2"))
    s.append(P("Site access is not required to obtain these. Send this "
               "protocol to someone on site:"))
    s.append(P("<i>Take 200 photos with a phone, held about 0.8 m above the "
               "floor (the robot camera height), from the walking aisle, aimed "
               "at the plant row. Repeat the same 10 positions at 4 different "
               "times of day, including late afternoon. Include 20 photos with "
               "tools, pipes or a coloured jacket in frame. Do not tidy the "
               "scene - the mess is the point. Note the time and the weather "
               "for each block.</i>", "note"))
    s.append(P("Annotate them with CVAT, Label Studio or Roboflow (all free at "
               "this size) and use them for fine-tuning and, crucially, as the "
               "<b>test set</b>. A score measured on rendered images does not "
               "predict a real greenhouse."))

    # ---------------------------------------------------------- 4 glass
    s.append(PageBreak())
    s.append(P("4. The glass warning - read before investing", "h1"))
    s.append(P("<b>Glass detection from RGB will not make the robot safe.</b> "
               "Glass has almost no appearance of its own: a model detects it "
               "from reflections, edge cues and the discontinuity between what "
               "is in front and behind, all of which change completely with "
               "viewpoint and lighting. Published glass segmenters report IoU "
               "in the 0.7 to 0.85 range on their own benchmarks and degrade "
               "sharply out of domain. That is a research result, not a "
               "protective device.", "warn"))
    s.append(P("For the robot's actual problem - do not drive into the pane - "
               "the engineering answer, in order:"))
    s.extend(BUL([
        "<b>Mark the glass physically.</b> An opaque band at lidar height "
        "(0.20 m) along every pane the robot can approach. Costs a roll of "
        "tape, works every time, and is what commissioning stage 3 will "
        "recommend.",
        "<b>Add a sensor that sees glass.</b> Ultrasonic rangefinders are "
        "cheap and reflect off glass, which lidar does not.",
        "<b>Bound the workspace in the map</b>, so the planner never routes "
        "within reach of an unmarked pane.",
    ]))
    s.append(P("Train the glass class as a <b>secondary cue and a report "
               "contribution</b> - it is legitimate and interesting work. Do "
               "not put it on the safety path."))

    # ---------------------------------------------------------- 5 workflow
    s.append(P("5. The workflow", "h1"))
    s.append(Preformatted(wrap_code(
        "# Install in a virtualenv, NOT into the ROS Python environment:\n"
        "# ultralytics pulls a torch build that will fight with ROS packages.\n"
        "python3 -m venv ~/venv-ml && source ~/venv-ml/bin/activate\n"
        "pip install -r ml/requirements.txt\n"
        "\n"
        "# 1. Merge every source into one YOLO tree with honest splits.\n"
        "python3 ml/prepare_dataset.py \\\n"
        "    --source ~/youbot_datasets/twin_morning:synthetic \\\n"
        "    --source ~/youbot_datasets/twin_evening:synthetic \\\n"
        "    --source ~/downloads/strawdi_yolo:real \\\n"
        "    --source ~/photos_serre_annotated:real \\\n"
        "    --out ~/strawberry_ds\n"
        "\n"
        "# 2. Widen the daylight range offline (recommended - see section 6).\n"
        "python3 ml/augment_daylight.py --dataset ~/strawberry_ds --split train\n"
        "\n"
        "# 3. Train.\n"
        "python3 ml/train.py --data ~/strawberry_ds/data.yaml \\\n"
        "    --model yolov8s --imgsz 960\n"
        "\n"
        "# 4. Turn predictions into label files the robot-side scorer reads.\n"
        "python3 ml/predict_to_labels.py \\\n"
        "    --weights runs/detect/strawberry/weights/best.pt \\\n"
        "    --images ~/strawberry_ds/test/images --out ~/preds/yolo\n"
        "\n"
        "# 5. The SAME script runs the current colour threshold, so the two\n"
        "#    are compared on identical images by identical code.\n"
        "python3 ml/predict_to_labels.py --baseline-colour \\\n"
        "    --images ~/strawberry_ds/test/images --out ~/preds/colour\n"
        "\n"
        "# 6. Score both.\n"
        "ros2 run youbot_commissioning stage8_detector -- \\\n"
        "    --labels   ~/strawberry_ds/test/labels \\\n"
        "    --predict  ~/preds/yolo \\\n"
        "    --baseline ~/preds/colour"), S["shell"]))

    s.append(P("Step 6 is the point of the whole exercise. <b>A model that "
               "does not beat the colour threshold on the same images has "
               "proved nothing.</b> Validation mAP from step 3 is for picking "
               "a checkpoint, not for reporting."))

    s.append(P("What prepare_dataset.py refuses to do", "h2"))
    s.extend(BUL([
        "<b>Split by image.</b> It splits by SOURCE GROUP. A robot at "
        "0.2 m/s photographs the same plant twenty times; splitting those at "
        "random puts near-duplicates in train and val, the validation score "
        "climbs, and it measures nothing.",
        "<b>Put synthetic frames in the test set.</b> The test split is drawn "
        "from real sources only, and the script exits with an error if every "
        "source is synthetic.",
        "<b>Concatenate class ids blindly.</b> Every public dataset numbers "
        "its classes from 0; a strawberry set's class 0 and a glass set's "
        "class 0 mean different things. A per-source remap is mandatory.",
    ]))

    # ---------------------------------------------------------- 6 settings
    s.append(P("6. The two settings that matter more than the model size", "h1"))
    s.append(P("6.1 Image size: use 960 or 1280, never 640", "h2"))
    s.append(P("Strawberries are small objects. A 34 mm berry at 2 m subtends "
               "about 15 px in a 640 px frame with this camera's 1.2 rad field "
               "of view, and the coarsest YOLO head has stride 32 - below "
               "roughly 20 px, recall falls off a cliff. Training at 960 px "
               "moves recall at range more than going from yolov8n to yolov8m "
               "does. It is the first thing to raise if distant fruit is being "
               "missed; do not blame the model size for it."))

    s.append(P("6.2 Hue augmentation: the default is wrong for this problem",
               "h2"))
    s.append(P("Ultralytics defaults to <font face='Courier'>hsv_h = 0.015</font>, "
               "a very narrow band of colour shift. <font face='Courier'>"
               "ml/train.py</font> raises it to 0.05, because the exact failure "
               "being fixed <i>is</i> a colour shift. Training with the default "
               "hue range on midday images reproduces the colour threshold's "
               "bug inside a neural network - a more expensive way to get the "
               "same failure."))
    s.append(P("<font face='Courier'>augment_daylight.py</font> goes further "
               "because a hue rotation is generic and this failure is not. "
               "What breaks the detector is a one-directional blackbody shift: "
               "sunlight through glass going warm, every surface moving the "
               "same way at once, until the G/R test cannot tell a warm scene "
               "from a red object. The script produces copies of the training "
               "set from overcast 7500 K to sunset 3000 K, gamma-correct so "
               "the shift lands across the tone range instead of piling up in "
               "the shadows. <b>Labels are copied verbatim: changing the "
               "illuminant moves no geometry, which is why this is free.</b>"))
    s.append(P("Sanity check before training: open a few "
               "<font face='Courier'>*_ctsunset_3000K</font> images. If the "
               "plants now look red to you, that is exactly the condition the "
               "colour threshold fails in - and the model has now seen it.",
               "note"))

    # ---------------------------------------------------------- 7 judging
    s.append(P("7. How the result will be judged", "h1"))
    s.append(TBL([
        ["Metric", "Target", "Why"],
        ["Precision", "at least 0.85",
         "The current detector produces roughly three spurious estimates out "
         "of five. Precision is the metric that recovers the 39% of harvest "
         "time lost to aligning on nothing."],
        ["Recall", "at least 0.80",
         "Current colour threshold: about 0.69, measured over all 127 berries "
         "including ones never in view, so the true figure is somewhat better."],
        ["F1 against the colour baseline", "strictly greater",
         "Measured on identical images by identical code. Without this the "
         "comparison proves nothing."],
        ["Annotated test images", "at least 100",
         "Real images only. A score on renders does not predict a greenhouse."],
        ["Missing prediction files", "zero",
         "A missing file silently becomes 'detected nothing' and inflates the "
         "false-negative count."],
    ], [46 * mm, 28 * mm, 94 * mm]))

    # ---------------------------------------------------------- 8 code
    s.append(PageBreak())
    s.append(P("8. Source", "h1"))
    s.append(P("Verbatim from the repository. This document is generated by "
               "<font face='Courier'>ml/make_handover_pdf.py</font> reading "
               "these files, so the listings are by construction the code that "
               "is in the tree. Regenerate after any change."))

    for f in ("classes.yaml", "requirements.txt", "prepare_dataset.py",
              "augment_daylight.py", "train.py", "predict_to_labels.py"):
        listing(f, s)

    doc.build(s)
    return OUT


if __name__ == "__main__":
    path = build()
    print(f"wrote {path} ({os.path.getsize(path) / 1024:.0f} kB)")
    sys.exit(0)
