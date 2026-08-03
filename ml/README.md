# Training a detector for strawberries, plants and glass

This directory is the offline half of the perception work: it does not run on
the robot and needs no ROS. It turns images into a model, and the model back
into YOLO label files that `youbot_commissioning`'s stage 8 can score.

**Read the glass warning in §3 before you spend a week on that class.**

---

## 0. Why this exists

The colour-threshold detector currently in the stack measures **69 % recall**
with **66 of 111** map estimates matching no real berry. It cannot be tuned out
of that. Its test is a channel ratio, `G < 0.33·R` and `B < 0.33·R`, which is
invariant to illumination **scale** — that is why it survives shade — but not
to illumination **colour**. Late-afternoon light through glass lowers `G/R` for
the entire scene, and at that point every surface passes the fruit test.

A learned model fixes the class of problem, not just the instance. But only if
it is trained on data that contains the failure.

---

## 1. What kind of data you need

### Annotation formats

| Class | Format | Why |
|---|---|---|
| `ripe_strawberry` | bounding box | that is what the mission acts on |
| `unripe_strawberry` | bounding box | hard negatives — see below |
| `strawberry_plant` | box, or mask if you can | foliage has no clean boundary |
| `glass` | **mask (segmentation)** | a pane is a region, not an object |

Everything is YOLO format: one `.txt` per image, lines of
`class cx cy w h`, all normalised to `[0,1]`. An **empty file** means "nothing
here, and I am sure" — it is a valid and necessary label. A **missing file**
means "unlabelled". They are not the same and confusing them inflates your
false-negative count silently.

### How much, and of what

Volume matters less than **coverage of the conditions that break the current
detector**. 500 images spanning a day beat 5 000 taken in one hour.

| Condition to cover | Why | Rough share |
|---|---|---|
| Morning, midday, late afternoon, overcast | the colour-temperature failure | 4 blocks, balanced |
| Both aisle sides (sunlit row, shaded row) | exposure extremes | 50/50 |
| Ripe, half-ripe, green fruit | ripeness is a continuum | all three present |
| Fruit occluded by leaves | **the majority case in reality** | ≥ 40 % of berries |
| Red objects that are *not* fruit: pipes, tools, crates, clothing | this is where false positives come from | ≥ 15 % of frames |
| Frames with **no** fruit at all | teaches "nothing here" | ~ 30 % of frames |
| Wet fruit / specular highlights | splits one berry into two blobs | some |
| Range 0.3 m to 2.5 m | small-object recall | spread |

A useful first target: **2 000 synthetic + 300–500 real annotated frames**.
The real ones matter far more per image; the synthetic ones make the model
stop overfitting to the few real backgrounds you have.

---

## 2. Where to get data

You cannot reach the greenhouse. Three sources, in the order you should use
them.

### 2.1 Your own digital twin — free, perfectly annotated, available tonight

This is the source you already own and the reason the twin exists. The world
knows where all 127 berries are, so it writes its own labels:

```bash
ros2 launch youbot_gazebo gazebo.launch.py
ros2 run youbot_slam dataset_capture --ros-args \
  -p session:=twin_morning -p target_frames:=2000
```

Output lands in `~/youbot_datasets/<session>/` as `images/`, `labels/`,
`index.csv` and `dataset.json`, already in YOLO format.

Labels come from the ground-truth catalogue through
`youbot_slam.lib.berry_view` — **never from the colour detector**. Training on
the detector's own output would teach the replacement to reproduce its
two-in-three false positives exactly.

Vary the conditions between sessions and record what you varied:

```bash
ros2 topic pub -1 /dataset/conditions std_msgs/String \
  "{data: 'warm light, sun low from the west, north row shaded'}"
```

**Honest limitation.** A model trained only on Gazebo renders transfers
poorly. Synthetic data is what stops you overfitting to three real
backgrounds; it is not a substitute for real images. Plan for fine-tuning.

### 2.2 Public datasets — real images, today, no site access

Names and descriptions below are from published work. **Verify the current URL
and the licence yourself before use** — hosting and terms change, and some are
research-only.

**Strawberries**

| Dataset | What it is | Notes |
|---|---|---|
| **StrawDI_Db1** | ~3 100 real strawberry images from fields in Huelva, Spain, with **instance segmentation masks** | The best single starting point. Masks convert to boxes trivially. Research use. |
| **DeepFruits** (Sa et al., *Sensors* 2016) | 7 fruit types incl. strawberry, boxes | Small but well documented; already cited in your report |
| **Roboflow Universe** — search "strawberry ripeness" / "strawberry detection" | Dozens of community sets, many with ripe/unripe splits | **Exports directly in YOLO format.** Quality varies a lot — inspect before trusting. Check each one's licence. |
| **Kaggle** — search "strawberry detection" | Several sets, mixed quality | Same caveat |

**Plants / foliage**

Honestly, the strawberry field datasets above already contain plants: annotate
the plant class on the same images rather than hunting for a separate source.
If you want more, **CVPPP** (Computer Vision Problems in Plant Phenotyping) has
leaf segmentation data, and **PlantVillage** has leaf imagery, though it is
disease-focused and shot on plain backgrounds — poor context for your problem.

**Glass**

| Dataset | What it is |
|---|---|
| **GDD** — Mei et al., *"Don't Hit Me! Glass Detection in Real-world Scenes"*, CVPR 2020 | ~3 900 images with glass region masks. The reference dataset for this task. |
| **Trans10K** — Xie et al., ECCV 2020 | ~10 400 images of transparent objects, two categories |
| **GSD** — Glass Surface Detection | Larger, more recent glass-surface data |

### 2.3 Real photographs of the actual greenhouse — the ones that matter most

You do not need to be there. Send someone on site this protocol:

> Take **200 photos with a phone**, holding it at about **0.8 m above the
> floor** (robot camera height), from the walking aisle, aimed at the plant
> row. Repeat the same 10 positions at **4 different times of day**, including
> late afternoon. Include 20 photos with tools, pipes or a coloured jacket in
> frame. Do not clean up the scene — mess is the point. Note the time and the
> weather for each block.

Then annotate them (CVAT, Label Studio, or Roboflow — all free for this size)
and use them for fine-tuning and, crucially, as the **test set**.

---

## 3. The glass warning — read before investing

**Glass detection from RGB will not make your robot safe.**

Glass has almost no appearance of its own. A model detects it from
reflections, edge cues, and the discontinuity between what is in front and
behind — all of which change completely with viewpoint and lighting. Published
glass segmenters report IoU in the 0.7–0.85 range on their own benchmarks, and
degrade sharply out of domain. That is a research result, not a protective
device.

For the robot's actual problem — *do not drive into the pane* — the correct
engineering answer, in order:

1. **Mark the glass physically.** An opaque band at lidar height (0.20 m)
   along every pane the robot can approach. Costs a roll of tape, works every
   time, and is what commissioning stage 3 will tell you to do.
2. **Add a sensor that sees glass**: ultrasonic rangefinders are cheap and
   reflect off glass, which lidar does not.
3. **Bound the workspace in the map**, so the planner never routes within
   reach of an unmarked pane.

Train the glass class if you want it as a **secondary cue and a report
contribution** — it is a legitimate and interesting piece of work. Do not put
it on the safety path.

---

## 4. The workflow

```bash
pip install -r requirements.txt

# 1. Merge every source into one YOLO tree with train/val/test splits.
python3 prepare_dataset.py \
    --source ~/youbot_datasets/twin_morning:synthetic \
    --source ~/youbot_datasets/twin_evening:synthetic \
    --source ~/downloads/strawdi_yolo:real \
    --source ~/photos_serre_annotated:real \
    --out ~/strawberry_ds

# 2. (optional but recommended) widen the daylight range offline.
python3 augment_daylight.py --dataset ~/strawberry_ds --split train

# 3. Train.
python3 train.py --data ~/strawberry_ds/data.yaml --model yolov8s --imgsz 960

# 4. Turn predictions into label files stage 8 can score.
python3 predict_to_labels.py \
    --weights runs/detect/train/weights/best.pt \
    --images ~/strawberry_ds/test/images \
    --out ~/preds/yolo

# 5. Score it, against the colour baseline, on identical images.
ros2 run youbot_commissioning stage8_detector -- \
    --labels   ~/strawberry_ds/test/labels \
    --predict  ~/preds/yolo \
    --baseline ~/preds/colour_threshold
```

Step 5 is the point of the whole exercise. A model that does not beat the
colour threshold **on the same images** has proved nothing.

---

## 5. Two settings that matter more than the architecture

**Image size.** Strawberries are small: a berry at 2 m subtends about 15 px in
a 640 px frame, and YOLO's stride-32 head struggles below ~20 px. Train at
`--imgsz 960` or `1280`. This single setting moves recall at range more than
switching model size does.

**Hue augmentation.** YOLO's default `hsv_h` is 0.015, which is far too narrow
for the exact failure this project has: illumination *colour* shifting over the
day. `train.py` raises it to 0.05 and explains why in the file. If you only
have midday images, this augmentation is what stands between you and a model
that fails at 17 h — the same way the colour threshold does.
